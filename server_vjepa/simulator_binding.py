#!/usr/bin/env python3
"""
simulator_binding.py
=================

Boilerplate / reference scaffold for "binding" the V-JEPA 2-AC world model
to a Simulator (MuJoCo/Gymnasium).

This adapts the logic from robot_binding.py to use a simulated environment
instead of real cameras and hardware.
"""

from __future__ import annotations

import dataclasses
import time
from typing import Optional

import numpy as np
import torch
import torch.nn.functional as F
import gymnasium as gym

# Import the core logic from robot_binding
try:
    from robot_binding import Config, VJEPAWorldModel, CEMPlanner
except ImportError:
    import sys
    import os
    sys.path.append(os.path.dirname(__file__))
    from robot_binding import Config, VJEPAWorldModel, CEMPlanner


# ==========================================================================
# SIMULATOR WRAPPER
# ==========================================================================

class SimulatorInterface:
    """Wraps a Gymnasium environment (e.g. MuJoCo Pusher-v4) to replace both
    the CameraInterface and the RobotArmController.
    """

    # ImageNet normalization used by V-JEPA
    MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
    STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)

    def __init__(self, config: Config, env_id: str = "Pusher-v4"):
        self.config = config
        self.env_id = env_id
        self.env = None
        self.last_frame = None
        
        # Override config action dim based on environment if needed.
        # Pusher-v4 has action space 7, perfectly matching our default config.
        self.config.action_dim = 7 

    def open(self) -> None:
        # Create env with rgb_array render mode to get camera observations
        self.env = gym.make(self.env_id, render_mode="rgb_array")
        obs, info = self.env.reset()
        self.last_frame = self.env.render()

    def close(self) -> None:
        if self.env is not None:
            self.env.close()
            self.env = None

    def capture_frame(self) -> np.ndarray:
        """Grab a single RGB frame from the simulator."""
        if self.env is None:
            raise RuntimeError("Environment not opened; call open() first")
        # Ensure we have a frame
        if self.last_frame is None:
             self.last_frame = self.env.render()
        
        # Gymnasium returns RGB frames directly, so we just return it.
        return self.last_frame

    def preprocess(self, frame_rgb: np.ndarray) -> torch.Tensor:
        """Convert a raw RGB frame into a normalized model input tensor."""
        import cv2
        h, w = self.config.image_size
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

    def denormalize_action(self, action: torch.Tensor) -> np.ndarray:
        """Map the planner's action vector to physical units for the simulator."""
        action_np = action.detach().cpu().numpy()
        # Scale to action bounds of the environment.
        # Gymnasium usually expects actions between -1.0 and 1.0 for continuous control.
        # We assume the model outputs actions in [-1, 1], so we can just use it directly
        # or apply any necessary scaling based on the specific environment limits.
        scale = np.ones(self.config.action_dim, dtype=np.float32)
        return action_np * scale

    def send_action(self, action: torch.Tensor) -> None:
        """Execute action in simulator."""
        if self.env is None:
            raise RuntimeError("Environment not opened; call open() first")
        physical_action = self.denormalize_action(action)
        
        # Take a step in the environment
        obs, reward, terminated, truncated, info = self.env.step(physical_action)
        
        # If the environment terminates, reset it
        if terminated or truncated:
            print("Environment terminated or truncated. Resetting...")
            self.env.reset()
            
        # Update our last frame cache
        self.last_frame = self.env.render()


# ==========================================================================
# MAIN ACTION-OBSERVATION LOOP (SIMULATED)
# ==========================================================================

def load_goal_latent(
    model: VJEPAWorldModel, simulator: SimulatorInterface, goal_image_path: Optional[str]
) -> torch.Tensor:
    """Obtain the goal latent the planner will steer toward."""
    if goal_image_path is not None:
        import cv2
        frame_bgr = cv2.imread(goal_image_path)
        if frame_bgr is None:
            raise FileNotFoundError(goal_image_path)
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        obs = simulator.preprocess(frame_rgb)
    else:
        obs = simulator.capture_observation()

    return model.encode_observation(obs)


def run_action_observation_loop(
    checkpoint_path: str,
    goal_image_path: Optional[str] = None,
    max_steps: int = 200,
    env_id: str = "Pusher-v4"
) -> None:
    """Main control loop binding simulator -> V-JEPA 2-AC -> simulator."""
    config = Config()

    simulator = SimulatorInterface(config, env_id=env_id)
    model = VJEPAWorldModel(config, checkpoint_path)
    planner = CEMPlanner(model, config)

    period_s = 1.0 / config.control_hz

    simulator.open()
    try:
        z_goal = load_goal_latent(model, simulator, goal_image_path)
        print(f"Goal loaded. Starting simulator loop for {max_steps} steps...")

        for step in range(max_steps):
            t0 = time.monotonic()

            # 1-2. Capture + encode current observation from simulator.
            obs_tensor = simulator.capture_observation()
            z_current = model.encode_observation(obs_tensor)

            # 3. Plan the next action toward the goal latent.
            next_action = planner.plan(z_current, z_goal)

            # 4. Execute on simulator.
            simulator.send_action(next_action)

            print(
                f"[step {step:04d}] action={next_action.cpu().numpy().round(4)}"
            )

            # 5. Maintain control rate (or run as fast as possible if simulating)
            # You may want to skip sleep in simulation to run faster, but keeping it
            # ensures timing matches real world expectation.
            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, period_s - elapsed))

    finally:
        simulator.close()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="V-JEPA 2-AC <-> simulator binding")
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
    parser.add_argument("--env-id", type=str, default="Pusher-v4", help="Gymnasium Environment ID")
    args = parser.parse_args()

    run_action_observation_loop(
        checkpoint_path=args.checkpoint,
        goal_image_path=args.goal_image,
        max_steps=args.max_steps,
        env_id=args.env_id
    )
