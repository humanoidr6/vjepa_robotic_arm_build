# V-JEPA 2 / 2.1 Checkpoints — What's Available and What Fits

Findings from actually probing `facebookresearch/vjepa2`, recorded so the next person
doesn't repeat the work. Verified 19 Jul 2026.

## Upstream bug: torch.hub downloads fail out of the box

`src/hub/backbones.py` in the vjepa2 repo ships with a local debug endpoint committed,
and the real URL commented out:

```python
# VJEPA_BASE_URL = "https://dl.fbaipublicfiles.com/vjepa2"
VJEPA_BASE_URL = "http://localhost:8300"
```

So **every** `torch.hub.load("facebookresearch/vjepa2", ...)` call dies with
`URLError: [Errno 111] Connection refused`. This is not a network problem on your end.

Fix the cached copy after the first (failed) load:

```bash
sed -i 's|^VJEPA_BASE_URL = "http://localhost:8300"|VJEPA_BASE_URL = "https://dl.fbaipublicfiles.com/vjepa2"|' \
  ~/.cache/torch/hub/facebookresearch_vjepa2_main/src/hub/backbones.py
```

Note this gets clobbered whenever the hub cache refreshes.

## Second upstream bug: wrong state-dict key, and an unvalidated ViT-B path

After fixing the URL, `_make_vjepa2_1_model` still fails:

```
KeyError: 'target_encoder'
```

The V-JEPA 2.1 checkpoints do not contain `target_encoder`. Their actual top-level keys are:

```
encoder, predictor, ema_encoder, opt, scaler, epoch, loss, batch_size, world_size, lr
```

Pass `checkpoint_key="ema_encoder"` instead. Note `opt` and `scaler` are **optimizer
state** — that is why these files are far larger than the weights alone, and it means a
slimmed re-save can shrink them dramatically.

With the right key the **encoder loads perfectly**, but the predictor then fails: the
function's defaults assume a 3072-dim teacher (ViT-G gigantic) while
`vjepa2_1_vitb_dist_vitG_384` was distilled from a 1664-dim teacher (ViT-g), and its
`predictor_embed` is a single Linear rather than a Sequential. `vjepa2_1_vit_base_384`
is not exported from `hubconf.py`, so this path appears never to have been validated.

That predictor is the **distillation** predictor and is not needed for our purposes.
Load encoder-only:

```python
import torch, sys
sys.path.insert(0, "~/.cache/torch/hub/facebookresearch_vjepa2_main")
from src.hub.backbones import _make_vjepa2_1_model, _clean_backbone_key

enc, _ = _make_vjepa2_1_model(model_name="vjepa2_1_vit_base_384", pretrained=False)
sd = torch.load(CKPT, map_location="cpu", weights_only=False)
enc.load_state_dict(_clean_backbone_key(sd["ema_encoder"]), strict=False)
```

### Verified working (19 Jul 2026, GTX 1660 Ti 6GB / 16GB RAM)

```
ENCODER LOADED: 86.8M params | fp16 174MB | missing=0 unexpected=0
forward (B,C,T,H,W) at 384px:
  T= 1 -> (1,  576, 768)
  T= 2 -> (1,  576, 768)
  T=16 -> (1, 4608, 768)
```

576 = (384/16)^2 spatial tokens per temporal step; `tubelet_size=2`, so T=1 and T=2
produce identical shapes and T=16 yields 8 temporal steps.

## Available checkpoints

From `ARCH_NAME_MAP` in `backbones.py`. Sizes are actual `content-length` from
`dl.fbaipublicfiles.com`, not parameter-count estimates — the files are substantially
larger than `params x 4 bytes`, so budget from this column.

| Checkpoint file | Download | Action-conditioned |
|---|---|---|
| `vjepa2-ac-vitg` | **11.76 GB** | **yes — the only one** |
| `vitg-384` | 16.46 GB | no |
| `vjepa2_1_vitl_dist_vitG_384` | 5.15 GB | no |
| `vitl` | 5.13 GB | no |
| `vjepa2_1_vitb_dist_vitG_384` | 1.66 GB | no |

`vjepa2_1_vit_base_384` and `vjepa2_1_vit_gigantic_384` are present in `ARCH_NAME_MAP`
but are **not** exported from `hubconf.py`. Reach them via
`src.hub.backbones._make_vjepa2_1_model(model_name=...)` directly.

## The constraint that matters for this project

**Action-conditioned post-training was only done at giant scale.** There is no small AC
model. That forces a choice:

- **Pretrained planning** → `vjepa2-ac-vitg`, 11.76 GB, ViT-g class. No lighter option.
- **Small encoder** (ViT-B 1.66 GB / ViT-L 5.15 GB) → representations only. You get no
  action conditioning, so you would have to train your own predictor on robot
  interaction data, which this project does not have yet.

### Hardware reality on the dev machine

- GPU: GTX 1660 Ti, **6 GB** VRAM
- System RAM: **14 GB total, ~9 GB typically available**

`torch.load` of the 11.76 GB AC checkpoint materializes it in system RAM before anything
reaches the GPU, and 11.76 GB does not fit in ~9 GB. Expect an OOM on load, not on
inference. Mitigations worth trying, in order: `torch.load(..., mmap=True)`, converting
to fp16 and re-saving once, or loading on a larger machine and shipping a slimmed
checkpoint.

## Slimming the AC checkpoint: 11.76 GB -> 2.64 GB (verified working)

The 11.76 GB `vjepa2-ac-vitg` does **not** need a big machine. Its actual contents:

| Key | Params | Needed for inference |
|---|---|---|
| `target_encoder` | 1012.2M | **yes** (EMA encoder) |
| `predictor` | 305.2M | **yes** |
| `encoder` | 1012.2M | no — non-EMA duplicate of `target_encoder` |
| `opt`, `scaler` | — | no — optimizer/AMP state |

Keeping only `target_encoder` + `predictor` and casting to fp16 gives **1317.4M params,
2.64 GB** — which fits a 6 GB laptop GPU and the Jetson Orin Nano's ~6 GB usable.

`mmap=True` is what makes this possible on a 16 GB machine: tensors page in lazily, so
peak RAM stays near the largest single tensor instead of 11.76 GB. Available RAM never
dropped below 9 GB during the conversion.

```python
import torch
sd = torch.load(SRC, map_location="cpu", mmap=True, weights_only=True)
out = {
    k: {n: (t.detach().clone().half() if t.is_floating_point() else t.detach().clone())
        for n, t in sd[k].items()}
    for k in ("target_encoder", "predictor")
}
torch.save(out, DST)
```

Note the AC checkpoint uses `target_encoder` (matching the loader's default), unlike the
2.1 checkpoints which use `ema_encoder`.

### Verified (19 Jul 2026, GTX 1660 Ti 6GB / 16GB RAM)

```
encoder:   missing=0 unexpected=0
predictor: missing=0 unexpected=0
encoder out (B,C,T,H,W)=(1,3,2,256,256) -> (1, 256, 1408)
predictor.forward: (x, actions, states, extrinsics=None)
```

## The AC predictor needs more than images

```python
predictor.forward(x, actions, states, extrinsics=None)
```

- `x` — encoded observation tokens
- `actions` — the candidate action sequence
- `states` — **proprioceptive robot state**
- `extrinsics` — camera pose relative to the robot (optional)

Two consequences this project must plan for:

**1. Proprioception.** The model expects to know where the arm currently *is*, not just
what the camera sees. The DS51150 is a standard PWM servo with **no position feedback** —
control is open-loop. The only available `states` signal is the *commanded* angle, which
diverges from truth exactly when it matters most: under load, at stall, and after the
gripper safety reflex freezes a joint. Options are to accept commanded-angle-as-state, add
encoders, or read the servo potentiometer directly (a hardware modification).

**2. Camera extrinsics.** Passing `extrinsics` means calibrating camera pose relative to
the arm base. Optional in the signature, but the model was trained with it.

## Interface mismatch with `robot_binding.py`

`VJEPAWorldModel` currently assumes:

```python
encode_observation(obs: (B, 3, H, W)) -> (B, latent_dim)   # one vector per still image
```

Real V-JEPA 2 is a **video** model. The encoder takes `(B, C, T, H, W)` clips and returns
**patch-token sequences** `(B, N, D)`, not a single pooled vector, and the AC predictor
consumes those tokens with temporal context rather than one latent per frame.

Adapting `robot_binding.py` therefore needs, at minimum:

1. A rolling frame buffer so a single camera frame becomes a `T`-frame clip.
2. A decision on tokens: keep `(B, N, D)` and score goal distance across tokens, or pool
   to `(B, D)`. Pooling is simpler and matches the existing `CEMPlanner`, but discards
   the spatial grounding that makes V-JEPA useful for manipulation.
3. A predictor call matching the AC signature, not `predictor(z, a)`.

The dummy stub hid all of this, because random-weight modules accept any shape you invent.

## Action space

V-JEPA 2-AC was post-trained on robot data with an **end-effector delta** action
convention, which is what `Config.action_dim = 7` (`[dx, dy, dz, drx, dry, drz, gripper]`)
already assumes. That is the good news.

The catch: converting an end-effector delta into the seven servo angles this arm actually
takes requires inverse kinematics, and IK requires a correct kinematic model. That is
blocked on the unresolved joint origins in `custom_arm/custom_arm.xml`. **The kinematics
blocker is on the critical path for the real arm** — though not for simulation, where the
environment supplies its own action space.
