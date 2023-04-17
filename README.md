# :four::nine::three::zero: 2023-Walter :robot:

### FRC 4930's 2023 Competitive Robot for FRC Charged Up!

## Robot Setup & Configuration
* Note: When the robot starts, make sure the arm is on the HOME position limit switch.

## Diver OI Controller Setup:
- [x] DRIVER Left & Right Stick: Translational and rotational robot movement.
- [x] DRIVER Left Trigger: While held, switch to virtual high gear.
- [x] DRIVER Right Trigger: While held, deploy intake to obtain a Cube.
- [x] DRIVER Left Bumper: While held, pick up cube with machine learning.
- [x] DRIVER Right Bumper: While held, close intake flappers.
- [x] DRIVER POV/D-Pad: Nudge robot (Left, Right, Up, Down) relative to the field.
- [x] DRIVER X (←) Button: Rotate chassis to +90-degree position relative to the field.
- [x] DRIVER Y (↑) Button: Rotate chassis to 0-degree position relative to the field.
- [x] DRIVER B (→) Button: Rotate chassis to -90-degree position relative to the field.
- [x] DRIVER A (↓) Button: Rotate chassis to 180-degree position relative to the field.
- [x] DRIVER Back (←) Button: Set swerve drive to a stationary X position.
- [x] DRIVER Start (→) Button: Reset gyro to a new field oriented forward position.

## Operator OI Controller Setup:
- [x] OPERATOR Left Stick: Spin gripper motors and rumble.
- [x] OPERATOR Right Stick: Direct control over the Arm.
- [x] OPERATOR Left Trigger: While held, lower the intake.
- [x] OPERATOR Right Trigger: High power Outtake.
- [x] OPERATOR Left Bumper: While held, open the gripper.
- [x] OPERATOR Right Bumper: Low power Outtake.
- [ ] OPERATOR POV/D-Pad: (UNASSIGNED)
- [x] OPERATOR X (←) Button: Go to position 00 and lock.
- [x] OPERATOR Y (↑) Button: Go to position 01 and lock.
- [x] OPERATOR B (→) Button: Go to position 02 and lock.
- [x] OPERATOR A (↓) Button: Go to position 03 and lock.
- [x] OPERATOR Start (→) Button: Reset max Pitch/Roll values on the dashboard.
- [x] OPERATOR Back (←) Button: Balance Values For Calibration

## CAN ID Assignments
* CAN ID 10 = SparkMax = Rear Right Drive Motor
* CAN ID 11 = SparkMax = Rear Right Turn Motor
* CAN ID 12 = SparkMax = Front Right Drive Motor
* CAN ID 13 = SparkMax = Front Left Turn Motor
* CAN ID 14 = SparkMax = Front Left Drive Motor
* CAN ID 15 = SparkMax = Front Left Turn Motor
* CAN ID 16 = SparkMax = Rear Left Drive Motor
* CAN ID 17 = SparkMax = Rear Left Turn Motor
* CAN ID 20 = SparkMax = Left Intake
* CAN ID 21 = SparkMax = Right Intake
* CAN ID 22 = SparkMax = Left Uprighter
* CAN ID 23 = SparkMax = Right Uprighter
* CAN ID 24 = SparkMax = Gripper
* CAN ID 25 = SparkMax = (UNASSIGNED)
* CAN ID 26 = SparkMax = Arm

## Solenoid ID Assignments
* Solenoid ID 1 = Intake Pistons
* Solenoid ID 2 = Lifter Pistons
* Solenoid ID 3 = Gripper Piston

## Built with:
* REV Robotics 3in MAXSwerve Modules https://www.revrobotics.com/rev-21-3005/
* OG Code (v2023.1) https://github.com/REVrobotics/MAXSwerve-Java-Template