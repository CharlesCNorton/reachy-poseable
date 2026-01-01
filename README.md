# reachy-poseable

Gravity compensation recorder for Reachy Mini. Move the robot by hand, record movements, play them back.

## Requirements

- Windows with miniconda/miniforge
- Reachy Mini robot
- Placo kinematics (installed via conda-forge)

## Installation

```bash
# Install miniconda, then:
conda install -c conda-forge pinocchio placo -y
pip install reachy-mini
```

## Usage

**Terminal 1 - Start daemon with Placo:**
```bash
set PYTHONIOENCODING=utf-8
python start_daemon_placo.py
```

**Terminal 2 - Run recorder:**
```bash
python placo_recorder.py
```

## Commands

| Key | Action |
|-----|--------|
| `c` | Connect to robot |
| `w` | Wake up (neutral position) |
| `g` | Enable gravity compensation (move by hand) |
| `s` | Go to sleep |
| `m` | Enable motor control |
| `o` | Disable all motors (limp) |
| `p` | Print current pose |
| `r` | Start/stop daemon recording |
| `R` | Manual record 5 seconds |
| `R10` | Manual record 10 seconds |
| `P` | Playback recording |
| `P0.5` | Playback at half speed |
| `S` | Save recording to file |
| `L` | Load recording from file |
| `l` | List saved recordings |
| `q` | Quit |

## Workflow

1. `c` - Connect
2. `w` - Wake up
3. `g` - Enable gravity comp (head + antennas become pliable)
4. Move the robot by hand to desired poses
5. `R5` - Record 5 seconds of movement
6. `P` - Play it back
7. `S` - Save to file
8. `s` - Put robot to sleep when done

## License

MIT
