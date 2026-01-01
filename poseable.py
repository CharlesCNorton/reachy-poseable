"""
Gravity compensation recorder for Reachy Mini.

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
import sys
from datetime import datetime
from enum import Enum, auto
from pathlib import Path
from typing import Optional, List, Dict, Any

import numpy as np

from reachy_mini import ReachyMini


class RobotState(Enum):
    """Current state of the robot."""
    DISCONNECTED = auto()
    MOTORS_ENABLED = auto()
    GRAVITY_COMP = auto()
    MOTORS_DISABLED = auto()
    SLEEPING = auto()


class ReachyRecorder:
    """Gravity compensation recorder for Reachy Mini."""

    def __init__(self):
        self.reachy: Optional[ReachyMini] = None
        self.recordings_dir = Path("recordings")
        self.recordings_dir.mkdir(exist_ok=True)
        self.current_recording: Optional[List[Dict[str, Any]]] = None
        self.is_recording = False
        self.state = RobotState.DISCONNECTED

    def _require_connected(self) -> bool:
        """Return True if connected, print error and return False if not."""
        if self.state == RobotState.DISCONNECTED:
            print("ERROR: Not connected. Use 'c' first.")
            return False
        return True

    def _require_pliable(self) -> bool:
        """Ensure robot is pliable (gravity comp or motors disabled). Auto-enable if needed."""
        if not self._require_connected():
            return False
        if self.state in (RobotState.GRAVITY_COMP, RobotState.MOTORS_DISABLED):
            return True
        print("Enabling gravity compensation...")
        try:
            self.reachy.enable_gravity_compensation()
            self.reachy.disable_motors(ids=["right_antenna", "left_antenna"])
            self.state = RobotState.GRAVITY_COMP
            print("Robot is now pliable.")
            time.sleep(0.2)
            return True
        except Exception as e:
            print(f"ERROR enabling gravity comp: {e}")
            return False

    def connect(self) -> bool:
        """Connect to Reachy Mini."""
        if self.state != RobotState.DISCONNECTED:
            print("Already connected.")
            return True
        try:
            print("Connecting...")
            self.reachy = ReachyMini(media_backend="no_media")
            self.reachy.__enter__()
            self.state = RobotState.MOTORS_ENABLED
            print("Connected.")
            return True
        except Exception as e:
            print(f"ERROR: {e}")
            self.reachy = None
            return False

    def disconnect(self):
        """Disconnect from Reachy Mini."""
        if self.is_recording:
            self.is_recording = False
        if self.reachy:
            try:
                self.reachy.__exit__(None, None, None)
            except:
                pass
            self.reachy = None
        self.state = RobotState.DISCONNECTED
        print("Disconnected.")

    def wake_up(self):
        """Wake up the robot to neutral position."""
        if not self._require_connected():
            return
        try:
            print("Waking up...")
            self.reachy.enable_motors()
            self.reachy.wake_up()
            self.state = RobotState.MOTORS_ENABLED
            print("Robot at neutral position.")
        except Exception as e:
            print(f"ERROR: {e}")

    def go_to_sleep(self):
        """Put robot to sleep position."""
        if not self._require_connected():
            return
        try:
            print("Going to sleep...")
            self.reachy.enable_motors()
            time.sleep(0.1)
            self.reachy.goto_sleep()
            self.state = RobotState.SLEEPING
            print("Robot asleep.")
        except Exception as e:
            print(f"ERROR: {e}")

    def enable_gravity_comp(self):
        """Enable gravity compensation - robot becomes pliable."""
        if not self._require_connected():
            return
        try:
            print("Enabling gravity compensation...")
            self.reachy.enable_gravity_compensation()
            self.reachy.disable_motors(ids=["right_antenna", "left_antenna"])
            self.state = RobotState.GRAVITY_COMP
            print("Head + antennas are now pliable.")
        except Exception as e:
            print(f"ERROR: {e}")

    def enable_motors(self):
        """Re-enable motor control (exit gravity comp mode)."""
        if not self._require_connected():
            return
        try:
            print("Enabling motors...")
            self.reachy.enable_motors()
            self.state = RobotState.MOTORS_ENABLED
            print("Motors enabled. Robot holds position.")
        except Exception as e:
            print(f"ERROR: {e}")

    def disable_motors(self):
        """Disable all motors - robot goes completely limp."""
        if not self._require_connected():
            return
        try:
            print("Disabling all motors...")
            self.reachy.disable_motors()
            self.state = RobotState.MOTORS_DISABLED
            print("All motors disabled. Robot is limp.")
        except Exception as e:
            print(f"ERROR: {e}")

    def record(self, duration: float = 5.0, frequency: float = 50.0):
        """Record robot positions for specified duration."""
        if not self._require_pliable():
            return None
        if self.is_recording:
            print("ERROR: Already recording!")
            return None

        self.is_recording = True
        print(f"\n>>> RECORDING {duration}s <<<")
        print(">>> Move the robot NOW <<<\n")

        frames = []
        start_time = time.time()
        interval = 1.0 / frequency
        last_sec = -1

        try:
            while (time.time() - start_time) < duration:
                loop_start = time.time()
                elapsed = loop_start - start_time

                # Progress each second
                sec = int(elapsed)
                if sec > last_sec:
                    last_sec = sec
                    print(f"  {duration - sec:.0f}s remaining...")

                try:
                    head_pose = self.reachy.get_current_head_pose()
                    head_joints, antenna_joints = self.reachy.get_current_joint_positions()
                    frame = {
                        "time": elapsed,
                        "head": head_pose.tolist(),
                        "antennas": list(antenna_joints),
                        "body_yaw": head_joints[0] if len(head_joints) > 0 else 0.0,
                    }
                    frames.append(frame)
                except Exception as e:
                    pass  # Skip read errors silently

                sleep_time = interval - (time.time() - loop_start)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\n  Interrupted.")

        self.is_recording = False

        if not frames:
            print("ERROR: No frames captured!")
            return None

        self.current_recording = frames
        print(f"\nRecorded {len(frames)} frames ({frames[-1]['time']:.1f}s)")
        return frames

    def playback(self, speed: float = 1.0):
        """Play back the current recording."""
        if not self._require_connected():
            return
        if not self.current_recording:
            print("ERROR: No recording. Use 'R' to record first.")
            return
        if speed <= 0 or speed > 10:
            print("ERROR: Speed must be between 0 and 10.")
            return

        data = self.current_recording
        duration = data[-1].get("time", len(data) * 0.02)
        print(f"\nPlaying {len(data)} frames at {speed}x ({duration/speed:.1f}s)")

        try:
            print("Enabling motors...")
            self.reachy.enable_motors()
            self.state = RobotState.MOTORS_ENABLED
            time.sleep(0.1)

            # Go to start position
            first = data[0]
            if "head" in first:
                print("Moving to start...")
                self.reachy.goto_target(
                    head=np.array(first["head"]),
                    antennas=first.get("antennas", [0.0, 0.0]),
                    duration=1.0
                )
                time.sleep(0.2)

            print("Playing...")
            start_time = time.time()
            idx = 0

            while idx < len(data):
                elapsed = (time.time() - start_time) * speed

                # Advance to correct frame
                while idx < len(data) - 1 and data[idx + 1].get("time", 0) < elapsed:
                    idx += 1

                frame = data[idx]
                if "head" in frame:
                    try:
                        self.reachy.set_target(
                            head=np.array(frame["head"]),
                            antennas=frame.get("antennas", [0.0, 0.0]),
                            body_yaw=frame.get("body_yaw", 0.0)
                        )
                    except:
                        pass

                if idx >= len(data) - 1:
                    break
                time.sleep(0.01)

            print("Playback complete.")

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
|              REACHY POSEABLE                          |
+-------------------------------------------------------+
|  c - Connect       w - Wake up      s - Sleep         |
|  d - Disconnect    g - Gravity comp m - Motors on     |
|                    o - Motors off   p - Print pose    |
|                                                       |
|  R   - Record 5s   P   - Playback   S - Save          |
|  R10 - Record 10s  P2  - Play 2x    L - Load          |
|  R30 - Record 30s  P.5 - Play 0.5x  l - List files    |
|                                                       |
|  h - Help          q - Quit                           |
+-------------------------------------------------------+
Prompt shows: [--]=disconnected [M]=motors [G]=gravity [Z]=sleep
"""

    print(menu)

    while True:
        try:
            # State indicator
            ind = {
                RobotState.DISCONNECTED: "--",
                RobotState.MOTORS_ENABLED: "M",
                RobotState.GRAVITY_COMP: "G",
                RobotState.MOTORS_DISABLED: "O",
                RobotState.SLEEPING: "Z",
            }.get(recorder.state, "?")

            cmd = input(f"\n[{ind}]> ").strip()

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

            # Recording (R, R5, R10, R30, etc.)
            elif cmd.upper().startswith("R"):
                duration = 5.0
                if len(cmd) > 1:
                    try:
                        duration = float(cmd[1:])
                        if duration <= 0 or duration > 300:
                            print("ERROR: Duration must be 1-300 seconds.")
                            continue
                    except ValueError:
                        print("ERROR: Use R, R5, R10, R30, etc.")
                        continue
                recorder.record(duration=duration)

            # Playback (P, P2, P0.5, etc.)
            elif cmd.upper().startswith("P"):
                speed = 1.0
                if len(cmd) > 1:
                    try:
                        speed = float(cmd[1:])
                    except ValueError:
                        print("ERROR: Use P, P2, P0.5, etc.")
                        continue
                recorder.playback(speed=speed)

            # File operations
            elif cmd == "S":
                recorder.save_recording()

            elif cmd == "L":
                recorder.list_recordings()
                filename = input("Filename: ").strip()
                if filename:
                    recorder.load_recording(filename)

            elif cmd == "l":
                recorder.list_recordings()

            elif cmd in ("h", "?"):
                print(menu)

            elif cmd == "q":
                print("Goodbye!")
                recorder.disconnect()
                break

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
