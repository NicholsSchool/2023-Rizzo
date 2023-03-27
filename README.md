# :four::nine::three::zero: 2023-Walter :robot:

### FRC 4930's 2023 Competitive Robot for FRC Charged Up!

## Robot Setup & Configuration
* Note: When the robot starts, make sure the arm is on the HOME position limit switch.

## Diver OI Controller Setup:
- [x] DRIVER Left Stick: Translational movement relative to the field.
- [x] DRIVER Right Stick: Rotational movement of the robot chassis.
- [x] DRIVER Left Trigger: While held, switch to virtual high gear.
- [x] DRIVER Right Trigger: While held, deploy intake to obtain a Cube.
- [x] DRIVER Left Bumper: Close intake flappers.
- [x] DRIVER Right Bumper: Open intake flappers.
- [x] DRIVER POV/D-Pad: Nudge (Left, Right, Up, Down) relative to the robot.
- [ ] DRIVER X (←) Button: Rotate chassis to +90-degree position relative to the field.
- [ ] DRIVER Y (↑) Button: Rotate chassis to 0-degree position relative to the field.
- [ ] DRIVER B (→) Button: Rotate chassis to -90-degree position relative to the field.
- [ ] DRIVER A (↓) Button: Rotate chassis to 180-degree position relative to the field.
- [x] DRIVER Start (→) Button: Reset gyro to a new field oriented forward position.
- [x] DRIVER Back (←) Button: Set swerve drive to a stationary X position.

## Operator OI Controller Setup:
- [x] OPERATOR Left Stick: Spin gripper motors.
- [x] OPERATOR Right Stick: Direct control over the Arm.
- [x] OPERATOR Left Trigger: While held, lower the intake.
- [x] OPERATOR Right Trigger: Outtake a Cube (intake, uprighter, gripper).
- [x] OPERATOR Left Bumper: Close gripper.
- [x] OPERATOR Right Bumper: Open gripper.
- [x] OPERATOR POV/D-Pad: Nudge (Left, Right, Up, Down) relative to the field.
- [x] OPERATOR X (←) Button: Go to HOME position and lock.
- [x] OPERATOR Y (↑) Button: Go to HUMAN PLAYER position and lock.
- [x] OPERATOR B (→) Button: Go to SCORING position and lock.
- [ ] OPERATOR A (↓) Button:
- [ ] OPERATOR Start (→) Button:
- [ ] OPERATOR Back (←) Button:

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