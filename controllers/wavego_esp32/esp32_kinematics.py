import math

# Wavego Walk Parameters (from Arduino ServoCtrl.h)
WALK_HEIGHT_MAX  = 110.0
WALK_HEIGHT_MIN  = 75.0
WALK_HEIGHT      = 95.0
WALK_LIFT        = 9.0
WALK_RANGE       = 40.0
WALK_ACC         = 5.0
WALK_EXTENDED_X  = 16.0
WALK_EXTENDED_Z  = 25.0
WALK_SIDE_MAX    = 30.0
WALK_MASS_ADJUST = 21.0
STAND_HEIGHT     = 95.0

WALK_LIFT_PROP   = 0.25

# Physical Leg Lengths for standard Serial IK (Webots URDF layout)
# Thigh length = 41.2mm, Calf length = 62.6mm
LEG_L1 = 41.2
LEG_L2 = 62.6

def singleLegCtrl(leg_num, xPos, yPos, zPos):
    # The ESP32 logic mirrors the local X axis for left-side legs (Leg 2 and 4).
    # We un-mirror it so X always points physically forward.
    if leg_num in [2, 4]:
        xPos = -xPos
        
    # Standard Analytical Inverse Kinematics for a 2-segment serial leg
    roll_angle = math.degrees(math.atan2(zPos, yPos))
    y_eff = math.hypot(yPos, zPos)
    
    d = math.hypot(xPos, y_eff)
    
    # Prevent IK math domain errors if target is unreachable
    d = min(d, LEG_L1 + LEG_L2 - 0.01)
    
    # By Law of Cosines:
    # Knee Pitch inside angle = acos((L1^2 + L2^2 - d^2) / 2*L1*L2)
    cos_inner = (LEG_L1*LEG_L1 + LEG_L2*LEG_L2 - d*d) / (2 * LEG_L1 * LEG_L2)
    cos_inner = max(-1.0, min(1.0, cos_inner))
    knee_inner_angle = math.acos(cos_inner)
    
    # Knee bend from straight (pi - inner angle)
    knee_angle = math.pi - knee_inner_angle
    
    # Hip Pitch
    # Angle from vertical to target
    theta_d = math.atan2(xPos, y_eff)
    # Angle from target vector to L1
    cos_alpha = (LEG_L1*LEG_L1 + d*d - LEG_L2*LEG_L2) / (2 * LEG_L1 * d)
    cos_alpha = max(-1.0, min(1.0, cos_alpha))
    alpha = math.acos(cos_alpha)
    
    # Assume knee bends "backwards" relative to hip, meaning hip swings forward
    hip_angle = theta_d + alpha
    
    return roll_angle, math.degrees(hip_angle), math.degrees(knee_angle)


def singleGaitCtrl(leg_num, statusInput, cycleInput, directionInput, extendedX, extendedZ):
    rDiection = directionInput * math.pi / 180.0
    rDist = 0
    yGait = 0

    if cycleInput < (1.0 - WALK_LIFT_PROP):
        thresh1 = (WALK_ACC / (WALK_ACC * 2 + WALK_RANGE * statusInput)) * (1.0 - WALK_LIFT_PROP)
        thresh2 = ((WALK_ACC + WALK_RANGE * statusInput) / (WALK_ACC * 2 + WALK_RANGE * statusInput)) * (1.0 - WALK_LIFT_PROP)
        
        if cycleInput <= thresh1:
            yGait = (WALK_HEIGHT - WALK_LIFT) + (cycleInput / thresh1) * WALK_LIFT
        elif thresh1 < cycleInput <= thresh2:
            yGait = WALK_HEIGHT
        elif cycleInput > thresh2:
            yGait = WALK_HEIGHT - ((cycleInput - thresh2) / ((1.0 - WALK_LIFT_PROP) - thresh2)) * WALK_LIFT

        rDist = (WALK_RANGE * statusInput / 2.0 + WALK_ACC) - (cycleInput / (1.0 - WALK_LIFT_PROP)) * (WALK_RANGE * statusInput + WALK_ACC * 2.0)
    else:
        yGait = WALK_HEIGHT - WALK_LIFT
        rDist = -(WALK_RANGE * statusInput / 2.0 + WALK_ACC) + ((cycleInput - (1.0 - WALK_LIFT_PROP)) / WALK_LIFT_PROP) * (WALK_RANGE * statusInput + WALK_ACC * 2.0)

    xGait = math.cos(rDiection) * rDist
    zGait = math.sin(rDiection) * rDist
    return singleLegCtrl(leg_num, xGait + extendedX, yGait, zGait + extendedZ)


def triangularGait(global_input, direction_angle, turn_cmd):
    # Returns a dictionary {LegNum: (wiggle, fore, back)}
    # LegNum mapping typical in WAVEGO: 1=FR, 2=FL, 3=RR, 4=RL
    stepB = global_input
    stepC = (global_input + 0.25)
    stepD = (global_input + 0.5)
    stepA = (global_input + 0.75)

    if stepA > 1.0: stepA -= 1.0
    if stepB > 1.0: stepB -= 1.0
    if stepC > 1.0: stepC -= 1.0
    if stepD > 1.0: stepD -= 1.0

    aInput = 0.0
    bInput = 0.0
    
    if global_input <= 0.25:
        adProp = global_input
        aInput = WALK_MASS_ADJUST - (adProp / 0.125) * WALK_MASS_ADJUST
        bInput = -WALK_MASS_ADJUST
    elif 0.25 < global_input <= 0.5:
        adProp = global_input - 0.25
        aInput = -WALK_MASS_ADJUST + (adProp / 0.125) * WALK_MASS_ADJUST
        bInput = -WALK_MASS_ADJUST + (adProp / 0.125) * WALK_MASS_ADJUST
    elif 0.5 < global_input <= 0.75:
        adProp = global_input - 0.5
        aInput = WALK_MASS_ADJUST - (adProp / 0.125) * WALK_MASS_ADJUST
        bInput = WALK_MASS_ADJUST
    else:
        adProp = global_input - 0.75
        aInput = -WALK_MASS_ADJUST + (adProp / 0.125) * WALK_MASS_ADJUST
        bInput = WALK_MASS_ADJUST - (adProp / 0.125) * WALK_MASS_ADJUST

    out = {}
    if not turn_cmd:
        out[1] = singleGaitCtrl(1, 1.0, stepA, direction_angle, WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput)
        out[4] = singleGaitCtrl(4, 1.0, stepD, -direction_angle, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput)
        out[2] = singleGaitCtrl(2, 1.0, stepB, direction_angle, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput)
        out[3] = singleGaitCtrl(3, 1.0, stepC, -direction_angle, WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput)
    elif turn_cmd == -1:
        out[1] = singleGaitCtrl(1, 1.5, stepA, 90.0, WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput)
        out[4] = singleGaitCtrl(4, 1.5, stepD, 90.0, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput)
        out[2] = singleGaitCtrl(2, 1.5, stepB, -90.0, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput)
        out[3] = singleGaitCtrl(3, 1.5, stepC, -90.0, WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput)
    elif turn_cmd == 1:
        out[1] = singleGaitCtrl(1, 1.5, stepA, -90.0, WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput)
        out[4] = singleGaitCtrl(4, 1.5, stepD, -90.0, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput)
        out[2] = singleGaitCtrl(2, 1.5, stepB, 90.0, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput)
        out[3] = singleGaitCtrl(3, 1.5, stepC, 90.0, WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput)
        
    return out

def stand(height=STAND_HEIGHT):
    out = {}
    out[1] = singleLegCtrl(1, WALK_EXTENDED_X, height, WALK_EXTENDED_Z)
    out[2] = singleLegCtrl(2, -WALK_EXTENDED_X, height, WALK_EXTENDED_Z)
    out[3] = singleLegCtrl(3, WALK_EXTENDED_X, height, WALK_EXTENDED_Z)
    out[4] = singleLegCtrl(4, -WALK_EXTENDED_X, height, WALK_EXTENDED_Z)
    return out
