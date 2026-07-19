# V-JEPA 2 Robotic Arm

A 6-axis servo arm driven by Meta's V-JEPA 2 / V-JEPA 2-AC world model, planning in
latent space toward a goal image rather than running a behaviour-cloned policy.

## Status

Working, unvalidated against hardware. The control stack is written; the arm is not yet built.

| Component | State |
|---|---|
| Teensy firmware | Written, **unflashed** — no Teensy attached, PlatformIO not installed |
| CEM planner, camera interface | Written |
| ZMQ transport (server ↔ Jetson) | Written |
| MuJoCo / Gymnasium binding | Written |
| World model checkpoint loader | **Stub** — loads random weights, so the pipeline currently plans against noise (`robot_binding.py:215`) |
| MuJoCo arm model | **Incomplete** — joint origins unresolved, see below |
| ROS2 workspace, Pi4 watchdog | Empty |

## Known blockers

**1. The MuJoCo model has no joint origins.** All six links sit at the world origin. The source
download is a print package — STLs each modeled about their own local origin, plus a slicer 3MF
holding bed positions — so no file carries the assembled pose. Resolving it needs a STEP file,
servo-horn bore detection, or calipers. See the header of `custom_arm/custom_arm.xml`.

**2. Servo travel variant is unconfirmed.** The DS51150 ships as 180° or 270°. The firmware assumes
180°. On a 270° unit every commanded angle is off by 1.5×.

**3. Compute does not match the plan.** `vjepa-robotic-arm-todo.md` assumes 2× A100 40GB for FSDP
training. The development machine has a single GTX 1660 Ti (6 GB). Inference and simulation are
feasible; the training phases are not.

## Hardware

Read [`wiring_diagram.md`](wiring_diagram.md) before connecting anything. The headline constraint:
the DS51150 is a **12 V, 8 A-stall** servo, so seven of them is a ~56 A bus. **Servo power must not
route through the PCA9685** — that board carries the PWM signal only.

[`hardware_architecture.md`](hardware_architecture.md) covers the compute tiers and where the touch
sensor sits in the action-observation loop.

## Bring-up

Firmware boots with servos **limp** and refuses motion until sent `HOME`, because the arm's rest
position at power-up is unknown and 165 kg·cm of torque would yank every joint to 90° instantly.

```
READY   -> position the arm near neutral by hand (servos back-drive when limp)
HOME    -> energize at 90 degrees and hold
a0,a1,a2,a3,a4,a5,a6   -> set joint targets in degrees, slew-limited
RELAX   -> return to limp at any time
```

## Layout

```
server_vjepa/              world model, CEM planner, ZMQ server, sim binding
teensy_firmware/           PlatformIO project for the Teensy 4.1
custom_arm/                MuJoCo model + STL meshes
wiring_diagram.md          power topology, pinout, bring-up order
hardware_architecture.md   compute tiers, touch sensor loop
```

The Python venv (`vjepa2_env`, ~6.7 GB) and model checkpoints are gitignored. Recreate with
`server_vjepa/setup.sh` and `requirements.lock.txt`.

## Mechanical design — attribution

The arm's printed parts are **"DS51150 servo 6-axis robot" by Gerhard_Nell**, purchased from
Cults3D: https://cults3d.com/en/3d-model/gadget/ds51150-servo-6-axis-robot

The STL and 3MF files under `custom_arm/Gerhard_Nell/` are that designer's work, not ours, and are
included here under the terms of that purchase. They are not covered by this repository's license
and are not offered for redistribution — if you want these parts, buy them from the designer.
Everything else in this repository is our own.
