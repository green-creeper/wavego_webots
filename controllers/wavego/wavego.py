"""wavego_esp32 controller."""

import sys
import math
from controller import Robot
import gait_generator

# Initialize the Robot instance
robot = Robot()

# get the time step of the current world
timestep = int(robot.getBasicTimeStep())

# Initialize Camera
camera = robot.getDevice("camera")
if camera:
    camera.enable(timestep)
    print("Camera enabled.")
else:
    print("Warning: Camera device not found!")

# Initialize motors
motors = {}
for leg in range(1, 5):
    for joint in ['servo', 'leg', 'foot']:
        name = f"{joint}_{leg}"
        motor = robot.getDevice(f"{name}_motor")
        if motor:
            motors[name] = motor
        else:
            print(f"Warning: Motor {name}_motor not found!")

# Empirical testing in Webots:
# 'servo_x' motor in URDF controls Hip Roll (Side/Spread)
# 'leg_x' motor in URDF controls Hip Pitch (Forward/Back)
# 'foot_x' motor in URDF controls Knee Pitch (Up/Down)
AXIS_MAPPING = {
    # Front Right
    'servo_1': -1.0, 'leg_1': 1.0,  'foot_1': 1.0,
    # Front Left
    'servo_2': 1.0,  'leg_2': 1.0,  'foot_2': 1.0,
    # Rear Right (Inverted URDF axes for Pitch)
    'servo_3': 1.0,  'leg_3': -1.0, 'foot_3': -1.0,
    # Rear Left (Inverted URDF axes for Pitch)
    'servo_4': -1.0, 'leg_4': -1.0, 'foot_4': -1.0,
}

def apply_leg_angles(leg_num, roll_rad, thigh_rad, knee_rad):
    """
    Applies the mathematical rad angles to the Webots motors.
    """
    if f"servo_{leg_num}" in motors:
        motors[f"servo_{leg_num}"].setPosition(roll_rad * AXIS_MAPPING[f"servo_{leg_num}"])
        
    if f"leg_{leg_num}" in motors:
        motors[f"leg_{leg_num}"].setPosition(thigh_rad * AXIS_MAPPING[f"leg_{leg_num}"])
        
    if f"foot_{leg_num}" in motors:
        motors[f"foot_{leg_num}"].setPosition(knee_rad * AXIS_MAPPING[f"foot_{leg_num}"])

t = 0.0
gait_speed = 0.5  # Cycles per second
step_input = 0.0

# Create the new Triangular Gait generator
# step_length is the total sweeping angle of the thigh
# step_height is the upward bending angle of the knee
gait = gait_generator.WavegoGait(step_length=0.6, step_height=0.5, stance_fraction=0.75)

print("Started clean Python Joint-Space triangular gait!")

while robot.step(timestep) != -1:
    # Get camera image to render it in the Webots UI
    if camera:
        _ = camera.getImage()

    # Calculate exactly what angle each joint should be at this global time!
    angles = gait.calculate_gait(global_time=step_input, forward_speed=1.0)
    
    # 2. Apply angles to physical motors
    for leg, (roll, thigh, knee) in angles.items():
        apply_leg_angles(leg, roll, thigh, knee)
        
    t += (timestep / 1000.0)
    step_input += (timestep / 1000.0) * gait_speed
    if step_input > 1.0:
        step_input -= 1.0
