"""Sample Webots controller for the humanoid sprint benchmark."""

from controller import Robot, Motion


class Sprinter(Robot):
    """Make the NAO robot run as fast as possible."""

    def initialize(self):
        """Get device pointers, enable sensors and set robot initial pose."""
        # This is the time step (ms) used in the motion file.
        self.timeStep = 40
        # Get pointers to the shoulder motors.
        self.RShoulderPitch = self.getDevice('RShoulderPitch')
        self.LShoulderPitch = self.getDevice('LShoulderPitch')
        # Move the arms down.
        self.RShoulderPitch.setPosition(1.1)
        self.LShoulderPitch.setPosition(1.1)

        # # Get pointers to the 12 motors of the legs (not used).
        # self.RHipYawPitch = self.getDevice('RHipYawPitch')
        # self.LHipYawPitch = self.getDevice('LHipYawPitch')
        self.RHipRoll = self.getDevice('RHipRoll')
        self.LHipRoll = self.getDevice('LHipRoll')
        self.RHipPitch = self.getDevice('RHipPitch')
        self.LHipPitch = self.getDevice('LHipPitch')
        self.RKneePitch = self.getDevice('RKneePitch')
        self.LKneePitch = self.getDevice('LKneePitch')
        self.RAnklePitch = self.getDevice('RAnklePitch')
        self.LAnklePitch = self.getDevice('LAnklePitch')
        self.RAnkleRoll = self.getDevice('RAnkleRoll')
        self.LAnkleRoll = self.getDevice('LAnkleRoll')
        self.LShoulderRoll = self.getDevice('LShoulderRoll')
        self.RShoulderRoll = self.getDevice('RShoulderRoll')
        # getting pointer to the 2 shoulder motors

        # Get pointer to the gyroscope and enable it
        self.gyro = self.getDevice('gyro')
        self.gyro.enable(self.timeStep)

        # # Get pointers to the onboard cameras (not used).
        # self.CameraTop = self.getDevice('CameraTop')
        # self.CameraBottom = self.getDevice('CameraBottom')
        
        # # Enable the cameras.
        # self.CameraTop.enable(self.timeStep)
        # self.CameraBottom.enable(self.timeStep)

    def stabilize(self):
        """Use gyroscope data to stabilize the robot."""
        gyro_data = self.gyro.getValues()
        roll = gyro_data[0]
        pitch = gyro_data[1]
        
        # Apply corrections based on gyroscope data to ankle and hip joints
        correction_factor = 0.001
        self.RAnkleRoll.setPosition(self.RAnkleRoll.getTargetPosition() - correction_factor * roll)
        self.LAnkleRoll.setPosition(self.LAnkleRoll.getTargetPosition() - correction_factor * roll)
        self.RHipRoll.setPosition(self.RHipRoll.getTargetPosition() - correction_factor * roll)
        self.LHipRoll.setPosition(self.LHipRoll.getTargetPosition() - correction_factor * roll)
        
        self.RAnklePitch.setPosition(self.RAnklePitch.getTargetPosition() - correction_factor * pitch)
        self.LAnklePitch.setPosition(self.LAnklePitch.getTargetPosition() - correction_factor * pitch)
        self.RHipPitch.setPosition(self.RHipPitch.getTargetPosition() - correction_factor * pitch)
        self.LHipPitch.setPosition(self.LHipPitch.getTargetPosition() - correction_factor * pitch)
        
    def move_joints_smooth(self, joints, target_positions, durations, start_times=None):
        """Move specified joints smoothly to their target positions."""
        if len(joints) != len(target_positions):
            raise ValueError("The number of joints must match the number of target positions.")
        
        if isinstance(durations, (int, float)):  # Single duration for all joints
            durations = [durations] * len(joints)
        if len(joints) != len(durations):
            raise ValueError("The number of joints must match the number of durations.")
        
        if start_times is None:  # Default start time for all joints is 0
            start_times = [0] * len(joints)
        elif isinstance(start_times, (int, float)):  # Single start time for all joints
            start_times = [start_times] * len(joints)
        if len(joints) != len(start_times):
            raise ValueError("The number of joints must match the number of start times.")

        steps_list = [int(duration / (self.timeStep / 1000)) for duration in durations]
        start_steps_list = [int(start_time / (self.timeStep / 1000)) for start_time in start_times]
        current_positions = [joint.getTargetPosition() for joint in joints]
        increments_list = [(target - current) / steps if steps > 0 else 0 for target, current, steps in zip(target_positions, current_positions, steps_list)]

        max_steps = max(start + steps for start, steps in zip(start_steps_list, steps_list))

        for step in range(max_steps):
            for joint, increment, steps, start_step in zip(joints, increments_list, steps_list, start_steps_list):
                if start_step <= step < start_step + steps:  # Move only if within the duration for this joint
                    current_position = joint.getTargetPosition() + increment
                    joint.setPosition(current_position)
            if self.step(self.timeStep) == -1:
                return

        # Ensure the final positions are set
        for joint, target_position in zip(joints, target_positions):
            joint.setPosition(target_position)
        self.step(self.timeStep)
        

    def pause(self, duration):
        """Pause for a specified duration in seconds."""
        steps = int(duration / (self.timeStep / 1000))  # convert duration to steps
        for _ in range(steps):
            if self.step(self.timeStep) == -1:
                return

    def run(self):
        """Play the forward motion and loop on the walking cycle."""   


        
        
        while True:
            self.move_joints_smooth(
            joints=[self.RAnkleRoll, self.LAnkleRoll, self.LKneePitch, self.RKneePitch, self.LHipPitch, self.RHipPitch, self.LAnklePitch, self.RAnklePitch],
            target_positions=[0.1, 0.1, 0.6, 0.6, -0.6, -0.6, -0.3, -0.3 ],
            durations=0.2
            )
            #first step
            self.move_joints_smooth(
                joints=[self.RAnklePitch, self.RKneePitch, self.RHipPitch, self.LAnklePitch],
                target_positions=[0.1, 0.4, -0.9, -0.4],
                durations=0.2
            )
            self.move_joints_smooth(
                joints=[self.RAnkleRoll, self.LAnkleRoll],
                target_positions=[-0.13, -0.13],
                durations=0.2
            )
            #second step
            self.move_joints_smooth(
                joints=[self.RAnklePitch, self.RKneePitch, self.RHipPitch, self.LAnklePitch, self.LKneePitch, self.LHipPitch, self.RAnkleRoll, self.LAnkleRoll],
                target_positions=[-0.2, 0.4, -0.6, -0.31, 0.6, -0.7 , 0.05, 0.05],
                durations=[0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.4, 0.4]
            )
            self.stabilize()
            self.pause(0.06)

            if self.step(self.timeStep) == -1:
                break


controller = Sprinter()
controller.initialize()
controller.run()
