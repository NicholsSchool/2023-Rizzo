# :four::nine::three::zero: 2023-Walter :robot:

### FRC 4930's 2023 Competitive Robot for FRC Charged Up!

## Diver OI Controller Setup:
* DRIVER Left Stick: Translational movement relative to the field.
* DRIVER Right Stick: Rotational movement of the chassis along the X-axis.
* DRIVER X (←) Button: Rotate to -90 degree Yaw relative to the field.
* DRIVER Y (↑) Button: Rotate to 0 degree Yaw relative to the field.
* DRIVER B (→) Button: Rotate to 90 degree Yaw relative to the field.
* DRIVER A (↓) Button: Rotate to 180 degree Yaw relative to the field.
* DRIVER Left Trigger: While held, switch to virtual high gear.
* DRIVER Right Trigger: While held, deploy and spin robot intake.
* DRIVER Left Bumper: Evasive left robot action button.
* DRIVER Right Bumper: Evasive right robot action button.
* DRIVER POV/D-Pad: Nudge (Left, Right, Up, Down) relative to the robot.
* DRIVER Back (←) Button: Reset the robot's field oriented forward position.
* DRIVER Start (→) Button: Toggle robot relative vs field orientated driving.

## Operator OI Controller Setup:
* OPERATOR Left Stick: Direct control over the Arm. Overrides arm locks.
* OPERATOR Right Stick: not used
* OPERATOR X (←) Button: Go to Arm position #1 and lock.
* OPERATOR Y (↑) Button: Go to Arm position #2 and lock.
* OPERATOR B (→) Button: Go to Arm position #3 and lock.
* OPERATOR A (↓) Button: Go to Arm position #4 and lock.
* OPERATOR Left Bumper: Cycle through Grid numbers 1-9 for object placement.
* OPERATOR Right Bumper: Cycle through Tier numbers 1-3 for object placement.
* OPERATOR Left Trigger: While held, auto pick up game object using ML/AI.
* OPERATOR Right Trigger: Release game object from Grabber.
* OPERATOR POV/D-Pad: Nudge (Left, Right, Up, Down) relative to the field.
* OPERATOR Back (←) Button: Toggle defensive X position and prevent driving.
* OPERATOR Start (→) Button: Cycle out all intake and grabber motors.

## CAN ID Assignments
* CAN ID 10 = Rear Right Drive Motor
* CAN ID 11 = Rear Right Turn Motor
* CAN ID 12 = Front Right Drive Motor
* CAN ID 13 = Front Left Turn Motor
* CAN ID 14 = Front Left Drive Motor
* CAN ID 15 = Front Left Turn Motor
* CAN ID 16 = Rear Left Drive Motor
* CAN ID 17 = Rear Left Turn Motor
* CAN ID 20 = Left Intake
* CAN ID 21 = Right Intake
* CAN ID 22 = Uprighter
* CAN ID 23 = Arm
* CAN ID 24 = Gripper

## Built with:
* REV Robotics 3in MAXSwerve Modules https://www.revrobotics.com/rev-21-3005/
* OG Code (v2023.1) https://github.com/REVrobotics/MAXSwerve-Java-Template
