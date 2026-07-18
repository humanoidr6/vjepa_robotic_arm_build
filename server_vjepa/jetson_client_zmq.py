#!/usr/bin/env python3
"""
jetson_client_zmq.py
=====================
V-JEPA 2-AC Edge Client (for Jetson Orin Nano)

This script captures camera observations and sends them over the network
to the Cortex server (A100s). It receives the planned action back and
sends it to the motor controller (Teensy).
"""

import argparse
import pickle
import time
import cv2
import zmq
import torch # using torch just for tensor typing in RobotArmController

# Import the local hardware drivers from robot_binding
try:
    from robot_binding import Config, CameraInterface, RobotArmController
except ImportError:
    import sys
    import os
    sys.path.append(os.path.dirname(__file__))
    from robot_binding import Config, CameraInterface, RobotArmController

def main():
    parser = argparse.ArgumentParser(description="V-JEPA ZMQ Client (Edge)")
    parser.add_argument("--server-ip", type=str, required=True, help="IP address of the V-JEPA server")
    parser.add_argument("--port", type=int, default=5555, help="Port of the V-JEPA server")
    parser.add_argument("--goal-image", type=str, default=None, help="Path to goal image to upload")
    args = parser.parse_args()

    config = Config()
    
    # Initialize Camera and Robot Arm
    camera = CameraInterface(config)
    arm = RobotArmController(config)
    
    # Initialize ZeroMQ
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    
    server_addr = f"tcp://{args.server_ip}:{args.port}"
    print(f"[Client] Connecting to {server_addr}...")
    socket.connect(server_addr)

    camera.open()
    arm.connect()

    try:
        # 1. Send Goal Image
        print("[Client] Acquiring goal image...")
        if args.goal_image is not None:
            goal_bgr = cv2.imread(args.goal_image)
            if goal_bgr is None:
                raise FileNotFoundError(f"Goal image {args.goal_image} not found.")
        else:
            print("[Client] No goal image provided. Using current camera frame as goal.")
            goal_bgr = camera.capture_frame()

        goal_rgb = cv2.cvtColor(goal_bgr, cv2.COLOR_BGR2RGB)
        
        print("[Client] Uploading goal to server...")
        socket.send(pickle.dumps({"type": "goal", "image": goal_rgb}))
        
        reply_bytes = socket.recv()
        reply = pickle.loads(reply_bytes)
        if reply.get("status") != "ok":
            print(f"[Client] Server rejected goal: {reply}")
            return
        
        print("[Client] Goal accepted. Starting control loop.")

        # 2. Main Control Loop
        period_s = 1.0 / config.control_hz
        step = 0
        
        while True:
            t0 = time.monotonic()

            # Capture current observation
            frame_bgr = camera.capture_frame()
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

            # Send observation to server
            socket.send(pickle.dumps({"type": "obs", "image": frame_rgb}))
            
            # Receive action
            reply_bytes = socket.recv()
            reply = pickle.loads(reply_bytes)

            if "error" in reply:
                print(f"[Client] Server error: {reply['error']}")
                time.sleep(1.0)
                continue
            
            action_np = reply.get("action")
            
            # Need to convert action back to a tensor for the arm controller (since it expects torch.Tensor)
            # or we can modify the denormalize_action in RobotArmController to handle both.
            # We'll just wrap it in a CPU tensor to keep it compatible with robot_binding.py
            action_tensor = torch.from_numpy(action_np).float()
            
            # Send to hardware
            arm.send_action(action_tensor)
            
            print(f"[step {step:04d}] Received action: {action_np.round(4)}")
            step += 1

            # Sleep to maintain control rate
            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, period_s - elapsed))

    except KeyboardInterrupt:
        print("[Client] Stopping.")
    finally:
        camera.close()
        arm.close()

if __name__ == "__main__":
    main()
