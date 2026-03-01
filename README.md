# Wavego Webots Simulation

A complete, physics-accurate simulation of the Waveshare Wavego robotic dog within the [Webots](https://cyberbotics.com/) simulation environment. 

This project allows developers to test high-level navigation, computer vision, and walking gaits in a safe, physics-enabled virtual environment before deploying code to the real physical Wavego robot (ESP32 / Raspberry Pi).

## 🌟 Acknowledgements

The robot URDF and 3D geometries in this repository are heavily based on the excellent work done in [WAVEGO_3D_MODEL by arnaucresp0](https://github.com/arnaucresp0/WAVEGO_3D_MODEL). The raw STL files have been adapted, optimized with explicit stable inertia matrices, and converted into Webots PROTO definitions for smooth and collision-accurate physics simulation.

## 🛠️ How It Works

### 1. The Robot Model (`Wavego.proto`)
The robot is fully modeled using accurate masses, centers of mass, and nested rotational joints representing the 12 independent servomotors (3 per leg: hip, upper leg, knee). 

**Physics Stability:** 
Simulating highly detailed STL collision meshes on extremely lightweight parts (e.g. 10g shins) often causes physics ODE engines to explode. The robot in this project uses perfectly fitted primitive collision geometries (`Sphere` and `Box`) paired with explicit inertia matrices (`[0.0001 0.0001 ... ]`) to guarantee absolute numerical stability. The robot refuses to jitter or collapse.

### 2. The Kinematics Engine (`gait_generator.py`)
To mimic the physical robot, we implemented a brand new, clean, Object-Oriented **Mathematical Joint-Space Gait Generator** in pure Python.
- It bypasses the parallel-linkage equations and complex Inverse Kinematics entirely.
- It accurately calculates a smooth `triangularGait` locomotion using explicit sine waves for the knee lift and linear interpolation for the thigh thrust.
- It directly maps the resulting angles into the Webots URDF model's physical constraints.

### 3. The Webots Controller (`wavego.py`)
This is the main interaction script. It acts as the bridge between the simulated ESP32 kinematics and the Webots simulation environment.
- It instantiates the Webots `Robot` API and camera devices.
- It continuously loops the gait cycle, querying the `gait_generator` module for the exact joint angles needed, and sends positional commands directly to the 12 virtual motors in real-time.

## 🚀 Getting Started

### Requirements
- [Webots R2023b or newer](https://cyberbotics.com/)
- Python 3.8+ (for Webots Python controllers)

### Running the Simulation
1. Clone this repository.
2. Open Webots.
3. Open the world file: `File -> Open World...` and select `worlds/apartment.wbt`.
4. The simulation will automatically load the `Wavego` robot and its `wavego` Python controller.
5. Click **Run** (Play) in Webots. 
6. The robot will stand up and automatically march forward using the simulated triangular gait! You can view its POV by double toggling the Camera overlay.

## 📁 Project Structure

* `/controllers/wavego/` - Contains the main running script `wavego.py` and the `gait_generator.py` module.
* `/worlds/` - Contains the `apartment.wbt` test environment where the robot is spawned.
* `/protos/` - Contains the Webots PROTO definition and the STLs `meshes/` for the Wavego dog.
* `RobotAssembly.SLDASM` & `.STEP` - The original raw SolidWorks / CAD geometries.
