# Simulation for USV local path planning using Potential field vector

![Python](https://img.shields.io/badge/python-3.13%2B-blue) ![License](https://img.shields.io/badge/license-MIT-green)

This is a small personnal project I have been working on. As a robotic engineering student, I wanted to use my theoritical knowledge on path planning and control for a practical use. 

This project aims to simulate a small Unmanned Surface Vehicle equipped with a GPS and an IMU. A physics moddel is made for the Surface system with fake drivers. 

The user can use the sensors data and implement algorithms to complete different tasks. 

<!-- Add a screenshot or a GIF of the simulation here:
![Simulation demo](docs/demo.gif) -->

## Current status

- [x] Fake drivers: Arduino (motors), GNSS (NMEA frames), IMU (magnetometer)
- [x] Simple physics model for the USV
- [x] Mission: go to waypoints, avoiding obstacles with a potential field
- [x] Live Matplotlib display and static potential field plot
- [ ] Global path planning, waypoint ordering, multiple USVs (see "The end goal")

## First task: Going to waypoints by avoiding obstacles.

The first mission that my USV is able to fulfill is to go to a set of waypoints ( by order for now ). It uses potential field vector to avoid random obstacles in its way. 

## The end goal

The end goal is to have a complete path planning using multiple USV to intercept multiple targets ( moving slowly in comparison to the USVs), each following a set of waypoint intelligently by using certain research algorithms. 

Ideas that I have for now: 
- Global path planning: A* or RRT*.
- Local, potential field vector
- Order waypoints using the nearest neigbourgh frist 


## Disclaimer

For a quick but temporary result to test the potential field vector algorithm, I got help from Claude to get the code for the graphical interface and the plot.py script. 

## Getting started

### Requirements

- Python 3.13 or newer
- [uv](https://docs.astral.sh/uv/) (recommended) or `pip`
- A graphical backend for Matplotlib (Tk or Qt). On Ubuntu: `sudo apt install python3-tk`

### Installation

```bash
git clone <repository-url>
cd Tactical-Navigation
uv sync
```

## Launch the project

```
python3 launch.py

```

With `uv`: `uv run python launch.py`

This opens two windows side by side: the live simulation (`main.py`) and the potential field plot (`plot.py`). Each one can also be started alone:

```bash
uv run python main.py
uv run python -m src.user_scripts.utilities.plot
```

Available missions: `wpts` (go to the waypoints) and `test` (drive straight, to check that the USV moves).

## Configuration

Everything is set in `src/user_scripts/config.py`:

| Parameter | Meaning |
|---|---|
| `dt` | Simulation time step (s) |
| `nb_of_USVs` | Number of USVs in the fleet |
| `dict_of_waypoints` | `{id: (x, y)}` in meters. If `None`, random waypoints are generated |
| `dict_of_obstacles` | `{id: (x, y, radius)}` in meters. If `None`, random obstacles are generated |
| `nb_of_waypoints`, `nb_of_obstacles` | Number of random waypoints / obstacles when no dictionary is given |

Conventions: positions are in meters, headings in radians measured from the x axis. The origin `(0, 0)` corresponds to `LAT0`, `LON0` in `fake_USV_driver_gps.py` (Paris by default).

## Project structure

```
.
├── main.py                  # Starts the simulation
├── launch.py                # Starts the simulation and the plot side by side
└── src/
    ├── software.py          # Creates the fleet, the map and the GUI, and starts the threads
    ├── fleet/               # USV state and physics, fleet manager
    ├── hardware/
    │   └── fake_USV_drivers/  # Simulated Arduino, GNSS and IMU
    ├── map/                 # Waypoints and obstacles
    ├── graphics/            # Matplotlib window
    └── user_scripts/        # Everything the user writes for the USV
        ├── config.py
        ├── src/             # Sensors, control, motion planning, missions
        └── utilities/       # Helper functions and the potential field plot
```

## How it works

1. **Physics thread:** advances every USV by one step of `dt` from its motor commands.
2. **Mission thread (one per USV):** reads the sensors (GNSS gives the position, the IMU gives the heading), computes a desired heading with the potential field, and converts it into left and right motor commands.
3. **Main thread:** the Matplotlib window reads the positions and redraws every `dt`.

## Known limitations

- The physics model is a simplified one (constants `K` and `L` are arbitrary) and the sensors have no noise yet.
- The speed estimate is taken from the motor command, not measured.
- A potential field can trap the USV in a local minimum, which is why some waypoints may fail (see the commented waypoint in `config.py`).
- Only one type of robot (USV) is supported for now.

## License

This project is released under the MIT License. See [LICENSE](LICENSE).