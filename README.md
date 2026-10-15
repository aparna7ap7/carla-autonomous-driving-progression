## 🎥 Visual Demonstrations

Here is the visual progression of the autonomous driving scripts (`cr_1` to `cr_7`):

### Basic Spawning & Control (`cr_1` to `cr_3`)
<table>
  <tr>
    <td align="center"><b>cr_1: Spawn</b></td>
    <td align="center"><b>cr_2: Tracking</b></td>
    <td align="center"><b>cr_3: Pygame & Plot</b></td>
  </tr>
  <tr>
    <td><img src="images_captured/cr_1.png" width="250"/></td>
    <td><img src="images_captured/cr_2.gif" width="250"/></td>
    <td><img src="images_captured/cr_3_back_view.gif" width="250"/></td>
  </tr>
</table>

### Autonomous Navigation & Hazard Avoidance (`cr_4` to `cr_7`)
<table>
  <tr>
    <td align="center"><b>cr_4: Waypoints</b></td>
    <td align="center"><b>cr_5: PID Speed</b></td>
    <td align="center"><b>cr_6: Stanley Steering</b></td>
    <td align="center"><b>cr_7: Obstacle Avoid</b></td>
  </tr>
  <tr>
    <td><img src="images_captured/cr_4.gif" width="200"/></td>
    <td><img src="images_captured/cr_5.gif" width="200"/></td>
    <td><img src="images_captured/cr_6.gif" width="200"/></td>
    <td><img src="images_captured/cr_7.gif" width="200"/></td>
  </tr>
</table>

## Project Structure

The project is organized into sequential examples (`cr_1` to `cr_7`) located in their respective folders, demonstrating increasing complexity:

* `cr_1/`: Basic vehicle spawn and fixed spectator setup[cite: 10].
* `cr_2/`: Constant vehicle throttle and smooth spectator following[cite: 10].
* `cr_3/`: Integration of Pygame camera display and basic Matplotlib plotting (X-coord, Speed)[cite: 10].
* `cr_4/`: Simple waypoint following with a basic 3-state speed controller[cite: 10].
* `cr_5/`: Waypoint following using a PID controller for speed regulation[cite: 10].
* `cr_6/`: Waypoint following with PID speed control and Stanley steering control, including path plotting[cite: 10].
* `cr_7/`: Adds obstacle detection (`sensor.other.obstacle`), spawns a static obstacle, and implements emergency braking logic, showing the obstacle on the plot[cite: 10].

* `tools/`: Contains reusable helper modules:
  * `pid_controller.py`: PID controller implementation[cite: 10].
  * `pygame_display.py`: Manages the Pygame camera view window[cite: 10].
  * `plotter_x.py`: Matplotlib plotter for X-coordinate and speed[cite: 10].
  * `plotter_path.py`: Matplotlib plotter for X-Y trajectory and speed[cite: 10].
  * `plotter_path_obstacle.py`: Matplotlib plotter including obstacle visualization[cite: 10].

## Installation

Clone the repository:

```bash
git clone https://github.com/aparna7ap7/carla-autonomous-driving-progression.git
cd carla-autonomous-driving-progression
```
Install the required dependencies:

```bash
pip install carla pygame matplotlib numpy
```

## Quick Start

Make sure you have a running CARLA simulator server compatible with your environment.

Launch the CARLA simulator server:

```bash
./CarlaUE4.sh
```

Run any project progression script from the examples directory:

```bash
python3 examples/cr_7.py
```


