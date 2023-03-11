package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.autonomous.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class RobotContainer {

  // Create subsystems
  private final SwerveDrive swerveDrive = new SwerveDrive();
  private final Gripper gripper = new Gripper();
  private final Arm arm = new Arm();
  private final Intake intake = new Intake();
  Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  // OI controllers
  CommandXboxController driverOI = new CommandXboxController(0);
  CommandXboxController operatorOI = new CommandXboxController(1);

  

  // Autonomous Commands
  private final DefaultAuto defaultAuto = new DefaultAuto(swerveDrive);

  /** Robot Container Constructor. */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    swerveDrive.setDefaultCommand(
        new RunCommand(
            () -> swerveDrive.drive(
                -MathUtil.applyDeadband(driverOI.getLeftY(), 0.07),
                -MathUtil.applyDeadband(driverOI.getLeftX(), 0.07),
                -MathUtil.applyDeadband(driverOI.getRightX(), 0.07),
                true),
            swerveDrive));
    
  }





  /** Define all button() to command() mappings. */
  private void configureButtonBindings() {

    // ################ DRIVER OI CONTROLLER CONFIGURATION ################

    // +DRIVER Left Stick: Translational movement relative to the field.
    // +DRIVER Right Stick: Rotational movement of the chassis along the X-axis.
    // DRIVER X Button: Rotate to -90 degree Yaw relative to the field.
    // DRIVER Y Button: Rotate to 0 degree Yaw relative to the field.
    // DRIVER B Button: Rotate to 90 degree Yaw relative to the field.
    // DRIVER A Button: Rotate to 180 degree Yaw relative to the field.
    // +DRIVER Left Trigger: While held, switch to virtual high gear.
    // DRIVER Right Trigger: While held, deploy and spin robot intake.
    // DRIVER Left Bumper: Evasive left robot action button.
    // DRIVER Right Bumper: Evasive right robot action button.
    // DRIVER POV/D-Pad: Nudge (Left, Right, Up, Down) relative to the robot.
    // +DRIVER Back Button: Reset the robot's field oriented forward position.
    // DRIVER Start Button: Toggle robot relative vs field orientated driving.

    // DRIVER Left Trigger: While held, switch to virtual high gear.
    driverOI.leftTrigger(0.25)
        .onTrue(new InstantCommand(() -> swerveDrive.setVirtualHighGear()))
        .onFalse(new InstantCommand(() -> swerveDrive.setVirtualLowGear()));

    // DRIVER Back Button: Reset the robot's field oriented forward position.
    driverOI.back().whileTrue(new RunCommand(() -> swerveDrive.resetFieldOrientedGyro(), swerveDrive));
    

    // Driver OI Controller Sample Mappings
    driverOI.a().onTrue(new InstantCommand(() -> System.out.println("Driver A")));
    driverOI.b().onTrue(new InstantCommand(() -> System.out.println("Driver B")));
    driverOI.x().onTrue(new InstantCommand(() -> System.out.println("Driver X")));
    driverOI.y().onTrue(new InstantCommand(() -> System.out.println("Driver Y")));
    driverOI.povLeft().onTrue(new InstantCommand(() -> System.out.println("Driver POV Left")));
    driverOI.povRight().onTrue(new InstantCommand(() -> System.out.println("Driver POV Right")));
    driverOI.povUp().onTrue(new InstantCommand(() -> System.out.println("Driver POV Up")));
    driverOI.povDown().onTrue(new InstantCommand(() -> System.out.println("Driver POV Down")));
    driverOI.leftTrigger().onTrue(new InstantCommand(() -> System.out.println("Driver Left Trigger")));
    driverOI.rightTrigger().onTrue(new InstantCommand(() -> System.out.println("Driver Right Trigger")));
    driverOI.leftBumper().onTrue(new InstantCommand(() -> System.out.println("Driver Left Bumper")));
    driverOI.rightBumper().onTrue(new InstantCommand(() -> System.out.println("Driver Right Bumper")));
    driverOI.back().onTrue(new InstantCommand(() -> System.out.println("Driver Back")));
    driverOI.start().onTrue(new InstantCommand(() -> System.out.println("Driver Start")));

    // ################ OPERATOR OI CONTROLLER CONFIGURATION ################

    // OPERATOR Left Stick: Direct control over the Arm. Overrides arm locks.
    // OPERATOR Right Stick: Direct control over the Uprighter.
    // OPERATOR X Button: Go to Arm position #1 and lock.
    // OPERATOR Y Button: Go to Arm position #2 and lock.
    // OPERATOR B Button: Go to Arm position #3 and lock.
    // OPERATOR A Button: Go to Arm position #4 and lock.
    // OPERATOR Left Bumper: Cycle through Grid numbers 1-9 for object placement.
    // OPERATOR Right Bumper: Cycle through Tier numbers 1-3 for object placement.
    // OPERATOR Left Trigger: While held, auto pick up game object using ML/AI.
    // OPERATOR Right Trigger: Release game object from Grabber.
    // OPERATOR POV/D-Pad: Nudge (Left, Right, Up, Down) relative to the field.
    // ~OPERATOR Back Button: Toggle defensive X position and prevent driving.
    // OPERATOR Start Button: Cycle out all intake and grabber motors.

    // OPERATOR Back Button: Toggle defensive X position and prevent driving.
    operatorOI.back().whileTrue(new RunCommand(() -> swerveDrive.setX(), swerveDrive));

    // Operator OI Controller Sample Mappings
    operatorOI.a().onTrue(new InstantCommand(() -> System.out.println("Operator A")));
    operatorOI.b().onTrue(new InstantCommand(() -> System.out.println("Operator B")));
    operatorOI.x().onTrue(new InstantCommand(() -> System.out.println("Operator X")));
    operatorOI.y().onTrue(new InstantCommand(() -> System.out.println("Operator Y")));
    operatorOI.povLeft().onTrue(new InstantCommand(() -> System.out.println("Operator POV Left")));
    operatorOI.povRight().onTrue(new InstantCommand(() -> System.out.println("Operator POV Right")));
    operatorOI.povUp().onTrue(new InstantCommand(() -> System.out.println("Operator POV Up")));
    operatorOI.povDown().onTrue(new InstantCommand(() -> System.out.println("Operator POV Down")));
    operatorOI.leftTrigger().onTrue(new InstantCommand(() -> System.out.println("Operator Left Trigger")));
    operatorOI.rightTrigger().onTrue(new InstantCommand(() -> System.out.println("Operator Right Trigger")));
    operatorOI.leftBumper().onTrue(new InstantCommand(() -> System.out.println("Operator Left Bumper")));
    operatorOI.rightBumper().onTrue(new InstantCommand(() -> System.out.println("Operator Right Bumper")));
    operatorOI.back().onTrue(new InstantCommand(() -> System.out.println("Operator Back")));
    operatorOI.start().onTrue(new InstantCommand(() -> System.out.println("Operator Start")));
  }

  /**
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return defaultAuto.runAutoSequence();
  }

}
