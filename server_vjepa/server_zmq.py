#!/usr/bin/env python3
"""
server_zmq.py
==============
V-JEPA 2-AC Cortex Server

This script runs the computationally heavy World Model and Planner.
It binds to a ZeroMQ socket and waits for observations from the edge (Jetson).

Protocol (Request/Reply):
- Client sends: dict with {'type': 'goal'|'obs', 'image': numpy_array}
- Server replies:
  - If type == 'goal': {'status': 'ok'} (latents saved internally)
  - If type == 'obs':  {'action': numpy_array}
"""

import argparse
import pickle
import time
import numpy as np
import torch
import zmq

from robot_binding import Config, VJEPAWorldModel, CEMPlanner

def main():
    parser = argparse.ArgumentParser(description="V-JEPA ZMQ Server")
    parser.add_argument("--checkpoint", type=str, default="./checkpoints/vjepa2_ac.pt")
    parser.add_argument("--port", type=int, default=5555)
    args = parser.parse_args()

    config = Config()
    model = VJEPAWorldModel(config, args.checkpoint)
    planner = CEMPlanner(model, config)

    # Initialize ZeroMQ
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind(f"tcp://*:{args.port}")
    
    print(f"[Server] Bound to tcp://*:{args.port}")
    print("[Server] Waiting for goal image from client...")

    z_goal = None

    while True:
        # Receive request
        req_bytes = socket.recv()
        try:
            req = pickle.loads(req_bytes)
        except Exception as e:
            print(f"[Server] Error decoding request: {e}")
            socket.send(pickle.dumps({"error": "Invalid format"}))
            continue

        req_type = req.get("type")
        frame_rgb = req.get("image")

        if frame_rgb is None or not isinstance(frame_rgb, np.ndarray):
            socket.send(pickle.dumps({"error": "Missing or invalid 'image'"}))
            continue

        # Preprocess the frame directly using the same logic as CameraInterface
        import cv2
        h, w = config.image_size
        frame_resized = cv2.resize(frame_rgb, (w, h), interpolation=cv2.INTER_AREA)
        frame_float = frame_resized.astype(np.float32) / 255.0
        
        # Standard normalization
        MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        frame_norm = (frame_float - MEAN) / STD

        chw = np.transpose(frame_norm, (2, 0, 1))
        obs_tensor = torch.from_numpy(chw).unsqueeze(0).float().to(config.device)
        
        # Encode
        latent = model.encode_observation(obs_tensor)

        if req_type == "goal":
            z_goal = latent
            print("[Server] Goal latent set. Ready for control loop.")
            socket.send(pickle.dumps({"status": "ok"}))
        
        elif req_type == "obs":
            if z_goal is None:
                socket.send(pickle.dumps({"error": "Goal not set yet"}))
                continue
            
            t0 = time.monotonic()
            next_action = planner.plan(latent, z_goal)
            t_plan = time.monotonic() - t0
            
            action_np = next_action.cpu().numpy()
            print(f"[Server] Planned action: {action_np.round(4)} in {t_plan:.3f}s")
            socket.send(pickle.dumps({"action": action_np}))
        
        else:
            socket.send(pickle.dumps({"error": f"Unknown type '{req_type}'"}))

if __name__ == "__main__":
    main()
