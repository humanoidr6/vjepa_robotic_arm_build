#!/usr/bin/env python3
"""
robot_binding.py
=================

Boilerplate / reference scaffold for "binding" a custom robotic arm to
Meta's V-JEPA 2-AC world model.

--------------------------------------------------------------------------
WHAT "BINDING" ACTUALLY MEANS FOR V-JEPA 2-AC
--------------------------------------------------------------------------
V-JEPA 2-AC is NOT a policy network that maps image -> action in a single
forward pass (like a behavior-cloning policy). It is a *world model*:

    1. An encoder f_theta(o_t) maps a camera observation o_t into a latent
       embedding z_t.
    2. An action-conditioned predictor g_phi(z_t, a_t) predicts the NEXT
       latent embedding z_{t+1} that would result from taking action a_t
       in the current state.
    3. Robot control is done via **planning**, not regression: you sample
       many candidate action sequences, roll them forward through the
       predictor entirely in latent space (no re-rendering of images),
       score each rollout by how close its final predicted latent is to
       the latent of a GOAL image, and execute the first action of the
       best-scoring sequence (a receding-horizon / MPC loop). This is
       typically done with the Cross-Entropy Method (CEM).

So "binding hardware to the model" means wiring up four independent
pieces that this script scaffolds as separate classes:

    CameraInterface        -> captures + preprocesses real observations
    VJEPAWorldModel         -> wraps the pretrained encoder + predictor
    CEMPlanner              -> turns (current latent, goal latent) into
                               a concrete action vector via planning
    RobotArmController      -> translates an abstract action vector into
                               real motor/serial commands for YOUR arm

--------------------------------------------------------------------------
IMPORTANT / WHAT YOU MUST FILL IN
--------------------------------------------------------------------------
- The exact import paths / class names for loading V-JEPA 2-AC weights
  depend on the checkpoint distribution you're using (e.g. the official
  facebookresearch/vjepa2 repo). The `VJEPAWorldModel._load_checkpoint`
  method below is a clearly marked STUB -- replace it with the actual
  model construction + `load_state_dict` calls for your checkpoint.
- The action space (dimensionality, units, normalization) must match
  whatever action-conditioning the checkpoint was trained with (often
  end-effector delta-pose + gripper, or joint deltas). Set
  `Config.action_dim` accordingly and keep `denormalize_action` in sync.
- `RobotArmController.translate_action_to_command` is a STUB. This is
  where you map a continuous action vector to your arm's actual
  protocol (serial bytes, CAN frames, ROS messages, etc).

This file is meant to run end-to-end with dummy/random tensors so you
can validate wiring and timing before dropping in real weights.
"""

from __future__ import annotations

import dataclasses
import time
from typing import Optional

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None  # camera capture will fail loudly if actually used

import torch
import torch.nn.functional as F

try:
    import serial  # pyserial
except ImportError:  # pragma: no cover
    serial = None  # motor controller will fail loudly if actually used


# ==========================================================================
# CONFIGURATION
# ==========================================================================

@dataclasses.dataclass
class Config:
    """Central config so every component agrees on shapes/units.

    Attributes:
        camera_index: OpenCV camera device index.
        image_size: (H, W) the encoder expects. V-JEPA-family encoders
            commonly expect square crops (e.g. 224 or 256).
        device: torch device string ("cuda", "mps", "cpu").
        action_dim: dimensionality of the action vector the predictor
            was trained with. Example here: 7 = [dx, dy, dz, drx, dry,
            drz, gripper] delta end-effector pose. CHANGE to match your
            checkpoint's action convention.
        planning_horizon: number of future steps the CEM planner
            rolls out in latent space before scoring against the goal.
        cem_iterations: number of CEM refit iterations per control step.
        cem_population: number of candidate action sequences sampled
            per CEM iteration.
        cem_elite_frac: fraction of top-scoring candidates used to
            refit the sampling distribution each iteration.
        control_hz: how often we run a full plan-and-act cycle.
        serial_port / serial_baud: transport for the real motor
            controller. Adjust to your microcontroller.
    """
    camera_index: int = 0
    image_size: tuple[int, int] = (224, 224)
    device: str = "cuda" if torch.cuda.is_available() else "cpu"

    action_dim: int = 7
    action_low: tuple[float, ...] = (-1.0,) * 7
    action_high: tuple[float, ...] = (1.0,) * 7

    planning_horizon: int = 8
    cem_iterations: int = 5
    cem_population: int = 256
    cem_elite_frac: float = 0.1

    control_hz: float = 2.0

    serial_port: str = "/dev/ttyUSB0"
    serial_baud: int = 115200


# ==========================================================================
# CAMERA / OBSERVATION CAPTURE
# ==========================================================================

class CameraInterface:
    """Captures raw frames and formats them the way V-JEPA 2 expects.

    The encoder was trained on ImageNet-style normalized RGB tensors of
    shape (C, H, W) in roughly [0, 1] range then normalized by dataset
    mean/std. Adjust `MEAN`/`STD` if your checkpoint used different
    normalization statistics.
    """

    # Standard ImageNet normalization -- verify against your checkpoint's
    # actual preprocessing config before trusting this for real control.
    MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
    STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)

    def __init__(self, config: Config):
        self.config = config
        self._cap: Optional["cv2.VideoCapture"] = None

    def open(self) -> None:
        if cv2 is None:
            raise RuntimeError("opencv-python is required for CameraInterface")
        self._cap = cv2.VideoCapture(self.config.camera_index)
        if not self._cap.isOpened():
            raise RuntimeError(
                f"Could not open camera at index {self.config.camera_index}"
            )

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def capture_frame(self) -> np.ndarray:
        """Grab a single raw BGR frame as an (H, W, 3) uint8 array."""
        if self._cap is None:
            raise RuntimeError("Camera not opened; call open() first")
        ok, frame_bgr = self._cap.read()
        if not ok:
            raise RuntimeError("Failed to read frame from camera")
        return frame_bgr

    def preprocess(self, frame_bgr: np.ndarray) -> torch.Tensor:
        """Convert a raw BGR frame into a normalized model input tensor.

        Returns:
            A (1, 3, H, W) float32 tensor on `self.config.device`, ready
            to be passed to `VJEPAWorldModel.encode_observation`.
        """
        h, w = self.config.image_size
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb, (w, h), interpolation=cv2.INTER_AREA)

        frame_float = frame_resized.astype(np.float32) / 255.0
        frame_norm = (frame_float - self.MEAN) / self.STD

        # HWC -> CHW, add batch dim
        chw = np.transpose(frame_norm, (2, 0, 1))
        tensor = torch.from_numpy(chw).unsqueeze(0).float()
        return tensor.to(self.config.device)

    def capture_observation(self) -> torch.Tensor:
        """Convenience wrapper: capture + preprocess in one call."""
        return self.preprocess(self.capture_frame())


# ==========================================================================
# V-JEPA 2-AC WORLD MODEL WRAPPER
# ==========================================================================

class VJEPAWorldModel:
    """Wraps the pretrained V-JEPA 2-AC encoder + action-conditioned predictor.

    This class intentionally isolates all model-specific code so that
    swapping checkpoints or upgrading the model version only requires
    editing this class, not the planner or robot controller.
    """

    def __init__(self, config: Config, checkpoint_path: str):
        self.config = config
        self.checkpoint_path = checkpoint_path
        self.encoder: torch.nn.Module
        self.predictor: torch.nn.Module
        self._load_checkpoint(checkpoint_path)

    def _load_checkpoint(self, checkpoint_path: str) -> None:
        """STUB: build the real encoder/predictor and load weights.

        Replace this method's body with the actual construction calls
        for your V-JEPA 2-AC distribution, e.g. (illustrative only --
        names will depend on the exact repo/release you use):

            from vjepa2.models import build_encoder, build_ac_predictor
            ckpt = torch.load(checkpoint_path, map_location="cpu")
            self.encoder = build_encoder(ckpt["encoder_config"])
            self.encoder.load_state_dict(ckpt["encoder_state_dict"])
            self.predictor = build_ac_predictor(ckpt["predictor_config"])
            self.predictor.load_state_dict(ckpt["predictor_state_dict"])

        Until you wire in real weights, this stub instantiates tiny
        placeholder modules purely so the rest of the pipeline (camera
        -> encode -> plan -> act) can be exercised end-to-end.
        """
        print(
            f"[VJEPAWorldModel] STUB: pretending to load checkpoint from "
            f"'{checkpoint_path}'. Replace _load_checkpoint() with real "
            f"model construction before deploying on hardware."
        )

        latent_dim = 384  # placeholder; match your checkpoint's embed dim

        class _DummyEncoder(torch.nn.Module):
            def __init__(self, out_dim: int):
                super().__init__()
                self.net = torch.nn.Sequential(
                    torch.nn.AdaptiveAvgPool2d((8, 8)),
                    torch.nn.Flatten(),
                    torch.nn.Linear(3 * 8 * 8, out_dim),
                )

            def forward(self, x: torch.Tensor) -> torch.Tensor:
                return self.net(x)

        class _DummyPredictor(torch.nn.Module):
            """Predicts z_{t+1} from (z_t, a_t)."""

            def __init__(self, latent_dim: int, action_dim: int):
                super().__init__()
                self.net = torch.nn.Sequential(
                    torch.nn.Linear(latent_dim + action_dim, latent_dim),
                    torch.nn.GELU(),
                    torch.nn.Linear(latent_dim, latent_dim),
                )

            def forward(self, z: torch.Tensor, a: torch.Tensor) -> torch.Tensor:
                return self.net(torch.cat([z, a], dim=-1))

        self.latent_dim = latent_dim
        self.encoder = _DummyEncoder(latent_dim).to(self.config.device).eval()
        self.predictor = _DummyPredictor(
            latent_dim, self.config.action_dim
        ).to(self.config.device).eval()

    @torch.no_grad()
    def encode_observation(self, obs_tensor: torch.Tensor) -> torch.Tensor:
        """Encode a preprocessed image tensor into a latent embedding z.

        Args:
            obs_tensor: (B, 3, H, W) tensor from CameraInterface.

        Returns:
            (B, latent_dim) embedding.
        """
        return self.encoder(obs_tensor)

    @torch.no_grad()
    def predict_next_latent(
        self, z_t: torch.Tensor, action: torch.Tensor
    ) -> torch.Tensor:
        """Roll the world model forward one step in latent space.

        Args:
            z_t: (B, latent_dim) current latent state.
            action: (B, action_dim) candidate action.

        Returns:
            (B, latent_dim) predicted latent at t+1. This is the core
            "imagination" step that lets us evaluate candidate action
            sequences without ever moving the real robot or rendering
            an image.
        """
        return self.predictor(z_t, action)

    @torch.no_grad()
    def rollout(
        self, z_t: torch.Tensor, action_sequence: torch.Tensor
    ) -> torch.Tensor:
        """Unroll the predictor over a full action sequence.

        Args:
            z_t: (B, latent_dim) starting latent.
            action_sequence: (B, T, action_dim) candidate action sequence.

        Returns:
            (B, latent_dim) final latent after T predicted steps.
        """
        z = z_t
        horizon = action_sequence.shape[1]
        for t in range(horizon):
            z = self.predict_next_latent(z, action_sequence[:, t, :])
        return z


# ==========================================================================
# CEM PLANNER (turns latents into a concrete action to execute)
# ==========================================================================

class CEMPlanner:
    """Cross-Entropy Method planner over the V-JEPA 2-AC latent world model.

    Given the current latent state and a goal latent (e.g. the encoding
    of a goal image), this samples candidate action sequences from a
    Gaussian, scores them by final-latent distance to the goal, refits
    the Gaussian to the elite (best-scoring) candidates, and repeats.
    Only the FIRST action of the best final sequence is returned/executed
    -- this is standard receding-horizon MPC, which makes the controller
    robust to the world model's compounding prediction error.
    """

    def __init__(self, model: VJEPAWorldModel, config: Config):
        self.model = model
        self.config = config
        self.action_low = torch.tensor(config.action_low, device=config.device)
        self.action_high = torch.tensor(config.action_high, device=config.device)

    def plan(self, z_current: torch.Tensor, z_goal: torch.Tensor) -> torch.Tensor:
        """Run CEM and return the next action to execute.

        Args:
            z_current: (1, latent_dim) latent of the live camera frame.
            z_goal: (1, latent_dim) latent of the goal/target image.

        Returns:
            (action_dim,) tensor -- the single next action to send to
            the robot controller this control step.
        """
        cfg = self.config
        horizon, pop, elite_frac = (
            cfg.planning_horizon,
            cfg.cem_population,
            cfg.cem_elite_frac,
        )
        n_elite = max(1, int(pop * elite_frac))

        # Initialize sampling distribution: mean 0, wide std, over the
        # full (horizon, action_dim) action-sequence shape.
        mean = torch.zeros(horizon, cfg.action_dim, device=cfg.device)
        std = torch.ones(horizon, cfg.action_dim, device=cfg.device) * 0.5

        z_current_batch = z_current.expand(pop, -1)

        best_sequence = mean.clone()

        for _ in range(cfg.cem_iterations):
            # Sample candidate action sequences ~ N(mean, std), then clip
            # to the valid action range.
            noise = torch.randn(pop, horizon, cfg.action_dim, device=cfg.device)
            candidates = mean.unsqueeze(0) + noise * std.unsqueeze(0)
            candidates = torch.max(
                torch.min(candidates, self.action_high), self.action_low
            )

            # Imagine the outcome of every candidate sequence purely in
            # latent space via the world model.
            z_final = self.model.rollout(z_current_batch, candidates)

            # Score = negative distance to goal latent (lower distance
            # is better -> higher score).
            dist = F.pairwise_distance(z_final, z_goal.expand(pop, -1))
            scores = -dist

            elite_idx = torch.topk(scores, n_elite).indices
            elite = candidates[elite_idx]

            mean = elite.mean(dim=0)
            std = elite.std(dim=0) + 1e-6
            best_sequence = mean.clone()

        # Receding horizon: only execute the first action of the plan.
        next_action = best_sequence[0]
        return next_action


# ==========================================================================
# ROBOT ARM CONTROLLER (hardware I/O -- this is where YOUR arm plugs in)
# ==========================================================================

class RobotArmController:
    """Translates abstract V-JEPA action vectors into real motor commands.

    This class owns the connection to your arm.
    """

    def __init__(self, config: Config):
        self.config = config
        self._mc = None

    def connect(self) -> None:
        from pymycobot.mycobot import MyCobot
        import time
        print("Connecting to MyCobot...")
        # Make sure to adjust port and baud if config is different
        self._mc = MyCobot('/dev/ttyAMA0', 1000000)
        time.sleep(1)
        self._mc.power_on()
        time.sleep(1)

    def close(self) -> None:
        if self._mc is not None:
            self._mc.release_all_servos()
            self._mc = None

    def denormalize_action(self, action: torch.Tensor) -> np.ndarray:
        """Map the planner's [-1, 1]-ish action vector to physical units."""
        action_np = action.detach().cpu().numpy()
        # MyCobot expects angles in degrees, roughly -150 to 150 for most joints
        # Let's map a 6-DoF action vector + 1-DoF gripper
        # We will treat the action as delta joint angles (degrees).
        scale = np.array([5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 50.0], dtype=np.float32)
        return action_np * scale

    def send_action(self, action: torch.Tensor) -> None:
        """Full pipeline: normalized action -> physical unit -> MyCobot command."""
        if self._mc is None:
            raise RuntimeError("Not connected; call connect() first")
        
        physical = self.denormalize_action(action)
        
        # current angles
        current_angles = self._mc.get_angles()
        if not current_angles or len(current_angles) != 6:
            current_angles = [0, 0, 0, 0, 0, 0]
            
        # compute new angles
        new_angles = [c + p for c, p in zip(current_angles, physical[:6])]
        
        # limit angles loosely
        new_angles = [max(-150, min(150, a)) for a in new_angles]
        
        self._mc.send_angles(new_angles, 50)
        
        # Gripper: physical[6] delta
        # But maybe we just set gripper based on absolute value if it's not a delta?
        # Let's ignore gripper for simplicity or just set a dummy value.
        pass



# ==========================================================================
# MAIN ACTION-OBSERVATION LOOP
# ==========================================================================

def load_goal_latent(
    model: VJEPAWorldModel, camera: CameraInterface, goal_image_path: Optional[str]
) -> torch.Tensor:
    """Obtain the goal latent the planner will steer toward.

    In practice you'd load a fixed goal image (a photo of the desired
    end state, e.g. "block on top of the target") from disk. As a
    fallback for boilerplate testing, this grabs one live frame and
    uses it as a (trivial, already-achieved) goal so the loop runs.
    """
    if goal_image_path is not None:
        if cv2 is None:
            raise RuntimeError("opencv-python is required to load goal image")
        frame_bgr = cv2.imread(goal_image_path)
        if frame_bgr is None:
            raise FileNotFoundError(goal_image_path)
        obs = camera.preprocess(frame_bgr)
    else:
        obs = camera.capture_observation()

    return model.encode_observation(obs)


def run_action_observation_loop(
    checkpoint_path: str,
    goal_image_path: Optional[str] = None,
    max_steps: int = 200,
) -> None:
    """Main control loop binding camera -> V-JEPA 2-AC -> robot arm.

    Each iteration:
        1. Capture the current camera frame (observation).
        2. Encode it into the current latent state z_t.
        3. Run CEM planning against the goal latent to get the next
           action to execute.
        4. Translate that action into motor commands and send it over
           serial.
        5. Sleep to maintain the configured control rate, then repeat.
    """
    config = Config()

    camera = CameraInterface(config)
    model = VJEPAWorldModel(config, checkpoint_path)
    planner = CEMPlanner(model, config)
    arm = RobotArmController(config)

    period_s = 1.0 / config.control_hz

    camera.open()
    arm.connect()
    try:
        z_goal = load_goal_latent(model, camera, goal_image_path)

        for step in range(max_steps):
            t0 = time.monotonic()

            # 1-2. Capture + encode current observation.
            obs_tensor = camera.capture_observation()
            z_current = model.encode_observation(obs_tensor)

            # 3. Plan the next action toward the goal latent.
            next_action = planner.plan(z_current, z_goal)

            # 4. Execute on real hardware.
            arm.send_action(next_action)

            print(
                f"[step {step:04d}] action={next_action.cpu().numpy().round(4)}"
            )

            # 5. Maintain control rate.
            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, period_s - elapsed))

    finally:
        arm.close()
        camera.close()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="V-JEPA 2-AC <-> robot arm binding")
    parser.add_argument(
        "--checkpoint",
        type=str,
        default="./checkpoints/vjepa2_ac.pt",
        help="Path to the V-JEPA 2-AC checkpoint file",
    )
    parser.add_argument(
        "--goal-image",
        type=str,
        default=None,
        help="Path to a goal image; omit to use a live frame as a trivial goal",
    )
    parser.add_argument("--max-steps", type=int, default=200)
    args = parser.parse_args()

    run_action_observation_loop(
        checkpoint_path=args.checkpoint,
        goal_image_path=args.goal_image,
        max_steps=args.max_steps,
    )
