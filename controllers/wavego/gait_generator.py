import math

class WavegoGait:
    """
    A simple, clean, and mathematically verifiable Triangular Gait Generator.
    This class generates walking angles directly in Joint Space, entirely bypassing
    the need for complex and error-prone Inverse Kinematics.
    
    The gait is structured around a "Duty Cycle", where each leg spends a fraction
    of the time pushing backward on the ground (Stance), and the rest of the time
    swinging forward in the air (Swing).
    """
    
    def __init__(self, step_length=0.4, step_height=0.4, stance_fraction=0.75):
        """
        :param step_length: The maximum angle (radians) the thigh swings forward/backwards.
        :param step_height: The maximum angle (radians) the knee bends upwards during swing.
        :param stance_fraction: The fraction of the gait cycle the foot is on the ground.
                                0.75 = Creep/Walk (3 legs on ground at a time).
                                0.50 = Trot (2 legs on ground at a time).
        """
        self.step_length = step_length
        self.step_height = step_height
        self.stance_fraction = stance_fraction
        
        # Leg Phasing: dictates the timing offset for each leg's cycle.
        # Creep gait: FL -> RR -> FR -> RL
        self.leg_phases = {
            1: 0.50,  # Front Right
            2: 0.00,  # Front Left
            3: 0.25,  # Rear Right
            4: 0.75   # Rear Left
        }

    def _triangle_wave(self, t, peak_time):
        """
        Generates a basic triangle wave that peaks at `peak_time`.
        t=0 -> 0.0
        t=peak_time -> 1.0
        t=1 -> 0.0
        """
        if t < peak_time:
            return t / peak_time
        else:
            return 1.0 - (t - peak_time) / (1.0 - peak_time)

    def calculate_leg_angles(self, leg_num, global_time):
        """
        Calculates the (roll, thigh_pitch, knee_pitch) for a specific leg.
        
        :param leg_num: 1, 2, 3, or 4
        :param global_time: The current gait cycle progress (0.0 to 1.0)
        :return: (roll_rad, thigh_rad, knee_rad)
        """
        # 1. Apply the leg's specific time offset
        local_time = (global_time + self.leg_phases[leg_num]) % 1.0
        
        roll_angle = 0.0
        thigh_angle = 0.0
        knee_angle = 0.0
        
        if local_time <= self.stance_fraction:
            # --- STANCE PHASE (Foot on ground, pushing robot forward) ---
            # Thigh moves linearly from +step_length/2 (front) to -step_length/2 (back)
            progress = local_time / self.stance_fraction
            thigh_angle = (self.step_length / 2.0) - (progress * self.step_length)
            
            # Knee stays extended (0.0)
            knee_angle = 0.0
            
        else:
            # --- SWING PHASE (Foot in air, swinging leg forward) ---
            # Time progress through the swing phase (0.0 to 1.0)
            swing_progress = (local_time - self.stance_fraction) / (1.0 - self.stance_fraction)
            
            # Thigh moves linearly from -step_length/2 (back) to +step_length/2 (front)
            thigh_angle = (-self.step_length / 2.0) + (swing_progress * self.step_length)
            
            # Knee bends upwards to clear the ground. It peaks in the middle of the swing.
            # A sine wave provides a much smoother curve than a triangle for the lift.
            knee_angle = math.sin(swing_progress * math.pi) * self.step_height

        return roll_angle, thigh_angle, knee_angle

    def calculate_gait(self, global_time, forward_speed=1.0, turn_speed=0.0):
        """
        Returns a dictionary of raw mechanical angles for all 4 legs.
        :param global_time: 0.0 to 1.0 looping cycle timer.
        :param forward_speed: Multiplier for the step_length (0 to 1+).
        :param turn_speed: Adjusts roll joints for steering.
        :return: { 1: (roll, thigh, knee), 2: (...), 3: (...), 4: (...) }
        """
        angles = {}
        # Temporarily scale step length by speed
        original_step = self.step_length
        self.step_length *= forward_speed
        
        for leg in range(1, 5):
            r, t, k = self.calculate_leg_angles(leg, global_time)
            
            # Simple skid-steer implementation could adjust 'r' (roll) based on turn_speed.
            # But for a simple straightforward gait, roll is 0.
            
            angles[leg] = (r, t, k)
            
        # Restore configuration
        self.step_length = original_step
        
        return angles
