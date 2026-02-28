"""wavego_esp32 controller."""

import sys
import math
from controller import Robot
import esp32_kinematics

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

# After empirical testing in Webots:
# 'servo_x' motor in URDF controls Hip Roll (Side/Spread) [IK 'wiggle']
# 'leg_x' motor in URDF controls Hip Pitch (Forward/Back) [IK 'fore']
# 'foot_x' motor in URDF controls Knee Pitch (Up/Down) [IK 'back']
# 
# IK outputs angle logic assuming positive is forward/out.
# We determine multipliers mathematically based on URDF layout:
# Leg 1 (Front Right), Leg 2 (Front Left), Leg 3 (Rear Right), Leg 4 (Rear Left)
AXIS_MAPPING = {
    # Front Right
    'servo_1': -1, 'leg_1': 1,  'foot_1': 1,
    # Front Left
    'servo_2': 1,  'leg_2': 1,  'foot_2': 1,
    # Rear Right (Inverted URDF axes for Pitch)
    'servo_3': 1,  'leg_3': -1, 'foot_3': -1,
    # Rear Left (Inverted URDF axes for Pitch)
    'servo_4': -1, 'leg_4': -1, 'foot_4': -1,
}

def apply_leg_angles(leg_num, wiggle_deg, fore_deg, back_deg):
    """
    Applies the raw ESP32 servo angles (in degrees) to the Webots motors (in radians).
    - wiggle_deg (Hip Roll, Side/Spread) maps to 'servo_x'
    - fore_deg (Hip Pitch, Forward/Back) maps to 'leg_x'
    - back_deg (Knee Pitch, Up/Down) maps to 'foot_x'
    """
    if f"servo_{leg_num}" in motors:
        val = math.radians(wiggle_deg) * AXIS_MAPPING[f"servo_{leg_num}"]
        motors[f"servo_{leg_num}"].setPosition(val)
        
    if f"leg_{leg_num}" in motors:
        val = math.radians(fore_deg) * AXIS_MAPPING[f"leg_{leg_num}"]
        motors[f"leg_{leg_num}"].setPosition(val)
        
    if f"foot_{leg_num}" in motors:
        val = math.radians(back_deg) * AXIS_MAPPING[f"foot_{leg_num}"]
        motors[f"foot_{leg_num}"].setPosition(val)

t = 0.0
gait_speed = 0.5  # Cycles per second
step_input = 0.0

# Initial stand up command parameters
target_height = esp32_kinematics.STAND_HEIGHT

# Get the baseline standing angles from the ESP32 math.
# Since the Webots URDF represents the robot ALREADY standing at motor angle 0.0,
# we must subtract this baseline from all future commands.
INITIAL_ANGLES = esp32_kinematics.stand(height=target_height)

while robot.step(timestep) != -1:
    # Get camera image to render it in the Webots UI
    if camera:
        _ = camera.getImage()

    # 1. Use the virtual ESP32 to calculate IK for walking forward
    # directionAngle = 0 (straight ahead), turnCmd = 0 (no turning)
    angles = esp32_kinematics.triangularGait(step_input, direction_angle=0, turn_cmd=0)
    
    # 2. Apply angles to physical motors
    for leg, (w, f, b) in angles.items():
        w0, f0, b0 = INITIAL_ANGLES[leg]
        w_out, f_out, b_out = w - w0, f - f0, b - b0
        apply_leg_angles(leg, w_out, f_out, b_out)
        
        # Give us a hint on Leg 1
        if leg == 1 and int(t*10) % 5 == 0:
             print(f"Leg 1 input cmds (dW, dF, dB): {w_out:5.1f}, {f_out:5.1f}, {b_out:5.1f}")
        
    t += (timestep / 1000.0)
    step_input += (timestep / 1000.0) * gait_speed
    if step_input > 1.0:
        step_input -= 1.0
