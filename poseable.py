"""
Placo-based movement recorder for Reachy Mini.

Features:
- Gravity compensation mode (move robot by hand)
- Record movements in real-time
- Playback recorded movements
- Save/load recordings to JSON files
- Interactive CLI menu

Requires: reachy_mini with placo_kinematics optional dependency
"""

import json
import time
import os
import sys
from datetime import datetime
from pathlib import Path
from typing import Optional, List, Dict, Any

import numpy as np

from reachy_mini import ReachyMini
from reachy_mini.utils import create_head_pose


class ReachyRecorder:
    """Interactive recorder for Reachy Mini using Placo kinematics."""

    def __init__(self):
        self.reachy: Optional[ReachyMini] = None
        self.recordings_dir = Path("recordings")
        self.recordings_dir.mkdir(exist_ok=True)
        self.current_recording: Optional[List[Dict[str, Any]]] = None
        self.is_recording = False

    def connect(self) -> bool:
        """Connect to Reachy Mini."""
        try:
            print("Connecting to Reachy Mini...")
            self.reachy = ReachyMini(
                media_backend="no_media",
            )
            self.reachy.__enter__()
            print("Connected! Ready for gravity compensation.")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False

    def disconnect(self):
        """Disconnect from Reachy Mini."""
        if self.reachy:
            try:
                self.reachy.__exit__(None, None, None)
            except:
                pass
            self.reachy = None
            print("Disconnected.")

    def wake_up(self):
        """Wake up the robot to neutral position."""
        if not self.reachy:
            print("Not connected!")
            return
        print("Waking up...")
        self.reachy.wake_up()
        print("Robot is awake and ready.")

    def go_to_sleep(self):
        """Put robot to sleep position."""
        if not self.reachy:
            print("Not connected!")
            return
        print("Re-enabling motors...")
        self.reachy.enable_motors()
        print("Going to sleep...")
        self.reachy.goto_sleep()
        print("Robot is asleep.")

    def enable_gravity_comp(self):
        """Enable gravity compensation mode - robot becomes compliant."""
        if not self.reachy:
            print("Not connected!")
            return
        print("Enabling gravity compensation...")
        self.reachy.enable_gravity_compensation()
        # Disable antenna motors so they're pliable too
        self.reachy.disable_motors(ids=["right_antenna", "left_antenna"])
        print("Gravity compensation ENABLED. Head + antennas are pliable.")

    def enable_motors(self):
        """Re-enable motor control (exit gravity comp mode)."""
        if not self.reachy:
            print("Not connected!")
            return
        print("Enabling motor control...")
        self.reachy.enable_motors()
        print("Motor control ENABLED. Robot holds position.")

    def disable_motors(self):
        """Disable all motors - robot goes limp."""
        if not self.reachy:
            print("Not connected!")
            return
        print("Disabling motors...")
        self.reachy.disable_motors()
        print("Motors DISABLED. Robot is limp.")

    def start_recording(self):
        """Start recording robot movements."""
        if not self.reachy:
            print("Not connected!")
            return
        if self.is_recording:
            print("Already recording!")
            return

        print("Starting recording...")
        print("Move the robot! Press Enter to stop recording.")
        self.reachy.start_recording()
        self.is_recording = True
        print("RECORDING... (move the robot now)")

    def stop_recording(self) -> Optional[List[Dict[str, Any]]]:
        """Stop recording and return the recorded data."""
        if not self.reachy:
            print("Not connected!")
            return None
        if not self.is_recording:
            print("Not currently recording!")
            return None

        print("Stopping recording...")
        self.is_recording = False

        try:
            data = self.reachy.stop_recording()
            if data:
                self.current_recording = data
                duration = len(data) * 0.02  # ~50Hz recording
                print(f"Recording stopped! Captured {len(data)} frames ({duration:.1f}s)")
                return data
            else:
                print("No data recorded.")
                return None
        except Exception as e:
            print(f"Error stopping recording: {e}")
            return None

    def manual_record(self, duration: float = 5.0, frequency: float = 50.0):
        """
        Manually record by polling current positions.
        Useful if daemon recording isn't working.
        """
        if not self.reachy:
            print("Not connected!")
            return

        print(f"Manual recording for {duration}s at {frequency}Hz...")
        print("Move the robot! Recording starts NOW.")

        frames = []
        start_time = time.time()
        interval = 1.0 / frequency

        while (time.time() - start_time) < duration:
            loop_start = time.time()

            try:
                head_pose = self.reachy.get_current_head_pose()
                head_joints, antenna_joints = self.reachy.get_current_joint_positions()

                frame = {
                    "time": time.time() - start_time,
                    "head": head_pose.tolist(),
                    "antennas": list(antenna_joints),
                    "head_joints": list(head_joints),
                    "body_yaw": head_joints[0] if head_joints else 0.0,
                }
                frames.append(frame)
            except Exception as e:
                print(f"Error reading position: {e}")

            # Maintain loop frequency
            elapsed = time.time() - loop_start
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        self.current_recording = frames
        print(f"Recorded {len(frames)} frames ({duration:.1f}s)")
        return frames

    def playback(self, recording: Optional[List[Dict[str, Any]]] = None, speed: float = 1.0):
        """Play back a recorded movement."""
        if not self.reachy:
            print("Not connected!")
            return

        data = recording or self.current_recording
        if not data:
            print("No recording to play back!")
            return

        print(f"Playing back {len(data)} frames at {speed}x speed...")
        print("Press Ctrl+C to stop.")

        # Make sure motors are enabled
        self.reachy.enable_motors()
        time.sleep(0.1)

        try:
            # Go to starting position first
            first_frame = data[0]
            if "head" in first_frame:
                head_pose = np.array(first_frame["head"])
                antennas = first_frame.get("antennas", [0.0, 0.0])
                print("Moving to start position...")
                self.reachy.goto_target(head=head_pose, antennas=antennas, duration=1.0)

            # Play through frames
            start_time = time.time()
            frame_idx = 0

            while frame_idx < len(data):
                elapsed = (time.time() - start_time) * speed

                # Find the right frame for current time
                while frame_idx < len(data) - 1 and data[frame_idx + 1].get("time", 0) < elapsed:
                    frame_idx += 1

                frame = data[frame_idx]

                if "head" in frame:
                    head_pose = np.array(frame["head"])
                    antennas = frame.get("antennas", [0.0, 0.0])
                    body_yaw = frame.get("body_yaw", 0.0)

                    self.reachy.set_target(
                        head=head_pose,
                        antennas=antennas,
                        body_yaw=body_yaw
                    )

                # Check if we've played all frames
                if frame_idx >= len(data) - 1:
                    break

                time.sleep(0.01)  # ~100Hz update rate

            print("Playback complete!")

        except KeyboardInterrupt:
            print("\nPlayback interrupted.")

    def save_recording(self, filename: Optional[str] = None):
        """Save current recording to JSON file."""
        if not self.current_recording:
            print("No recording to save!")
            return

        if not filename:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"recording_{timestamp}.json"

        filepath = self.recordings_dir / filename

        # Create metadata
        save_data = {
            "description": input("Enter description (optional): ") or "Recorded movement",
            "recorded_at": datetime.now().isoformat(),
            "num_frames": len(self.current_recording),
            "time": [f.get("time", i * 0.02) for i, f in enumerate(self.current_recording)],
            "set_target_data": [
                {
                    "head": f.get("head", np.eye(4).tolist()),
                    "antennas": f.get("antennas", [0.0, 0.0]),
                    "body_yaw": f.get("body_yaw", 0.0),
                }
                for f in self.current_recording
            ]
        }

        with open(filepath, "w") as f:
            json.dump(save_data, f, indent=2)

        print(f"Saved to: {filepath}")

    def load_recording(self, filename: str) -> Optional[List[Dict[str, Any]]]:
        """Load a recording from JSON file."""
        filepath = self.recordings_dir / filename
        if not filepath.exists():
            # Try as absolute path
            filepath = Path(filename)

        if not filepath.exists():
            print(f"File not found: {filename}")
            return None

        with open(filepath, "r") as f:
            data = json.load(f)

        # Convert to playback format
        if "set_target_data" in data:
            # Standard format from SDK
            times = data.get("time", [])
            frames = []
            for i, frame_data in enumerate(data["set_target_data"]):
                frame = {
                    "time": times[i] if i < len(times) else i * 0.02,
                    "head": frame_data.get("head", np.eye(4).tolist()),
                    "antennas": frame_data.get("antennas", [0.0, 0.0]),
                    "body_yaw": frame_data.get("body_yaw", 0.0),
                }
                frames.append(frame)
            self.current_recording = frames
        else:
            # Raw frame format
            self.current_recording = data if isinstance(data, list) else data.get("frames", [])

        print(f"Loaded {len(self.current_recording)} frames from {filepath.name}")
        return self.current_recording

    def list_recordings(self):
        """List all saved recordings."""
        recordings = list(self.recordings_dir.glob("*.json"))
        if not recordings:
            print("No recordings found.")
            return

        print("\nSaved recordings:")
        print("-" * 50)
        for rec in sorted(recordings):
            try:
                with open(rec) as f:
                    data = json.load(f)
                desc = data.get("description", "No description")
                frames = data.get("num_frames", len(data.get("set_target_data", [])))
                print(f"  {rec.name}: {frames} frames - {desc}")
            except:
                print(f"  {rec.name}: (unable to read)")
        print("-" * 50)

    def get_current_pose(self):
        """Print current robot pose."""
        if not self.reachy:
            print("Not connected!")
            return

        try:
            head_pose = self.reachy.get_current_head_pose()
            head_joints, antenna_joints = self.reachy.get_current_joint_positions()

            print("\nCurrent Robot State:")
            print("-" * 40)
            print(f"Head pose (4x4 matrix):\n{head_pose}")
            print(f"\nHead joints: {[f'{j:.3f}' for j in head_joints]}")
            print(f"Antenna joints: {[f'{j:.3f}' for j in antenna_joints]}")
            print(f"Body yaw: {head_joints[0]:.3f} rad ({np.rad2deg(head_joints[0]):.1f} deg)")
            print("-" * 40)
        except Exception as e:
            print(f"Error reading pose: {e}")


def interactive_menu():
    """Run interactive CLI menu."""
    recorder = ReachyRecorder()

    menu = """
+-------------------------------------------------------+
|         REACHY MINI GRAVITY COMP RECORDER             |
+-------------------------------------------------------+
|  CONNECTION                                           |
|    c  - Connect                                       |
|    d  - Disconnect                                    |
|                                                       |
|  ROBOT CONTROL                                        |
|    w  - Wake up (go to neutral position)              |
|    s  - Go to sleep                                   |
|    g  - Enable GRAVITY COMPENSATION (move by hand)    |
|    m  - Enable motor control (hold position)          |
|    o  - Disable motors (go limp)                      |
|    p  - Print current pose                            |
|                                                       |
|  RECORDING                                            |
|    r  - Start/stop daemon recording                   |
|    R  - Manual record (5 seconds)                     |
|    R10- Manual record (10 seconds, e.g. R10, R30)     |
|                                                       |
|  PLAYBACK                                             |
|    P  - Playback current recording                    |
|    P2 - Playback at 2x speed (e.g. P0.5, P2)          |
|                                                       |
|  FILE OPERATIONS                                      |
|    S  - Save current recording                        |
|    L  - Load recording from file                      |
|    l  - List saved recordings                         |
|                                                       |
|    q  - Quit                                          |
+-------------------------------------------------------+
"""

    print(menu)

    while True:
        try:
            cmd = input("\n> ").strip()

            if not cmd:
                continue

            # Connection
            elif cmd == "c":
                recorder.connect()

            elif cmd == "d":
                recorder.disconnect()

            # Robot control
            elif cmd == "w":
                recorder.wake_up()

            elif cmd == "s":
                recorder.go_to_sleep()

            elif cmd == "g":
                recorder.enable_gravity_comp()

            elif cmd == "m":
                recorder.enable_motors()

            elif cmd == "o":
                recorder.disable_motors()

            elif cmd == "p":
                recorder.get_current_pose()

            # Recording
            elif cmd == "r":
                if recorder.is_recording:
                    recorder.stop_recording()
                else:
                    recorder.start_recording()
                    input("Press Enter to stop recording...")
                    recorder.stop_recording()

            elif cmd.startswith("R"):
                # Manual recording with optional duration
                duration = 5.0
                if len(cmd) > 1:
                    try:
                        duration = float(cmd[1:])
                    except:
                        pass
                recorder.manual_record(duration=duration)

            # Playback
            elif cmd.startswith("P"):
                speed = 1.0
                if len(cmd) > 1:
                    try:
                        speed = float(cmd[1:])
                    except:
                        pass
                recorder.playback(speed=speed)

            # File operations
            elif cmd == "S":
                recorder.save_recording()

            elif cmd == "L":
                recorder.list_recordings()
                filename = input("Enter filename to load: ").strip()
                if filename:
                    recorder.load_recording(filename)

            elif cmd == "l":
                recorder.list_recordings()

            elif cmd == "q":
                print("Goodbye!")
                recorder.disconnect()
                break

            elif cmd == "?":
                print(menu)

            else:
                print(f"Unknown command: {cmd}. Type ? for help.")

        except KeyboardInterrupt:
            print("\n(Ctrl+C detected)")
            if recorder.is_recording:
                recorder.stop_recording()

        except Exception as e:
            print(f"Error: {e}")


if __name__ == "__main__":
    interactive_menu()
