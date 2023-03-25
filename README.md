# :four::nine::three::zero: 2023-Walter :robot:

### FRC 4930's 2023 Competitive Robot for FRC Charged Up!

## Robot Setup & Configuration
* Note: When the robot starts, make sure the arm is on the HOME position limit switch.

## Diver OI Controller Setup:
* +DRIVER Left Stick: Translational movement relative to the field.
* +DRIVER Right Stick: Rotational movement of the robot chassis.
* +DRIVER Left Trigger: While held, switch to virtual high gear.
* DRIVER Right Trigger: While held, deploy intake to pickup a Cube.
* DRIVER Left Bumper: Open intake flappers.
* DRIVER Right Bumper: Close intake flappers.
* DRIVER POV/D-Pad: Nudge (Left, Right, Up, Down) relative to the robot.
* DRIVER X (←) Button: Rotate to LEFT position relative to the field.
* DRIVER Y (↑) Button: Rotate to FORWARDS position relative to the field.
* DRIVER B (→) Button: Rotate to RIGHT position relative to the field.
* DRIVER A (↓) Button: Rotate to BACKWARDS position relative to the field.
* DRIVER Start (→) Button: Reset gyro to a new field oriented forward position.
* DRIVER Back (←) Button: Set swerve drive to a stationary X position.

## Operator OI Controller Setup:
* OPERATOR Left Stick: Spin gripper motors.
* OPERATOR Right Stick: Direct control over the Arm.
* OPERATOR Left Trigger: While held, lower the intake.
* OPERATOR Right Trigger: While held, outtake Cube (intake, uprighter, gripper).
* OPERATOR Left Bumper: Open gripper.
* OPERATOR Right Bumper: Close gripper.
* OPERATOR POV/D-Pad: Nudge (Left, Right, Up, Down) relative to the field.
* OPERATOR X (←) Button: Go to HOME position and lock.
* OPERATOR Y (↑) Button: Go to HUMAN PLAYER position and lock.
* OPERATOR B (→) Button: Go to SCORING position and lock.
* OPERATOR A (↓) Button: Go to GROUND/INTAKE position and lock.
* OPERATOR Start (→) Button:
* OPERATOR Back (←) Button:

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
* CAN ID 25 = SparkMax = Arm

## Solenoid ID Assignments
* Solenoid ID 1 = Intake Pistons
* Solenoid ID 2 = Lifter Pistons
* Solenoid ID 3 = Gripper Piston

## Built with:
* REV Robotics 3in MAXSwerve Modules https://www.revrobotics.com/rev-21-3005/
* OG Code (v2023.1) https://github.com/REVrobotics/MAXSwerve-Java-Template