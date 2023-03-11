# :four::nine::three::zero: 2023-Walter :robot:

### FRC 4930's 2023 Competitive Robot for FRC Charged Up!

## Diver OI Controller Setup:
* DRIVER Left Stick: Translational movement of the robot relative to the field.
* DRIVER Right Stick: Rotational movement of the robot chassis using the X-axis.
* DRIVER X (←) Button: Rotate to -90 degree Yaw relative to the field.
* DRIVER Y (↑) Button: Rotate to 0 degree Yaw relative to the field.
* DRIVER B (→) Button: Rotate to 90 degree Yaw relative to the field.
* DRIVER A (↓) Button: Rotate to 180 degree Yaw relative to the field.
* DRIVER Left Bumper: Evasive left robot action button.
* DRIVER Right Bumper: Evasive right robot action button.
* DRIVER Left Trigger: While held, set to virtual high gear. Set to low gear when not held down.
* DRIVER Right Trigger: While held, deploy and spin robot intake. Otherwise retract intake.
* DRIVER POV/D-Pad: Nudge the robot in a given direction (Left, Right, Up, Down) relative to the robot.
* DRIVER Back (←) Button: Set the robot's field oriented forward position relative to the intake.
* DRIVER Start (→) Button: Toggle robot relative movement vs field orientated driving.

## Operator OI Controller Setup:
* OPERATOR Left Stick: Direct control over the Arm. Overrides locking mechanism.
* OPERATOR Right Stick: n/a
* OPERATOR X (←) Button: Go to Arm position #1 and lock.
* OPERATOR Y (↑) Button: Go to Arm position #2 and lock.
* OPERATOR B (→) Button: Go to Arm position #3 and lock.
* OPERATOR A (↓) Button: Go to Arm position #4 and lock.
* OPERATOR Left Bumper: Cycle through Grid numbers 1-9 for game object placement.
* OPERATOR Right Bumper: Cycle through Tier numbers 1-3 for game object placement.
* OPERATOR Left Trigger: While held, auto pick up game object using machine learning.
* OPERATOR Right Trigger: Release game object from Grabber.
* OPERATOR POV/D-Pad: Nudge the robot in a given direction (Left, Right, Up, Down) relative to the field.
* OPERATOR Back (←) Button: Defensive X position and prevent driving (useful for charging station).
* OPERATOR Start (→) Button: Cycle out all intake and grabber motors (useful to expel game objects.

## CAN ID Assignments
* 10 Rear Right Drive Motor
* 11 Rear Right Turn Motor
* 12 Front Right Drive Motor
* 13 Front Left Turn Motor
* 14 Front Left Drive Motor
* 15 Front Left Turn Motor
* 16 Rear Left Drive Motor
* 17 Rear Left Turn Motor
* 20 Left Intake
* 21 Right Intake
* 22 Uprighter
* 23 Arm
* 24 Gripper

## Built with:
* REV Robotics 3in MAXSwerve Modules https://www.revrobotics.com/rev-21-3005/
* OG Code (v2023.1) https://github.com/REVrobotics/MAXSwerve-Java-Template
