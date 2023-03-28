package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import static frc.robot.Constants.ArmConstants.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.networktables.GenericEntry;

public class RobotContainer {

  // Subsystems
  private final SwerveDrive swerveDrive;
  private final Gripper gripper;
  public final Arm arm;
  private final Intake intake;
  private final Uprighter uprighter;

  // Shuffleboard
  ShuffleboardTab walterTab;
  public static GenericEntry armPos;
  public static GenericEntry armLimit;
  public static GenericEntry gripperLimit;
  ComplexWidget autoChooserWidget;

  // OI controllers
  public static CommandXboxController driverOI;
  public static CommandXboxController operatorOI;
  public static XboxController driverRumbler;
  public static XboxController operatorRumbler;

  // Autonomous Chooser
  public static SendableChooser<Command> autoChooser;

  public RobotContainer() {

    // Instantiate all subsystems
    swerveDrive = new SwerveDrive();
    gripper = new Gripper();
    arm = new Arm();
    intake = new Intake();
    uprighter = new Uprighter();

    // OI (Operator Interface) Controllers & Rumblers
    driverOI = new CommandXboxController(1);
    driverRumbler = new XboxController(1);
    operatorOI = new CommandXboxController(0);
    operatorRumbler = new XboxController(0);

    // EXAMPLE: driverRumbler.setRumble(RumbleType.kBothRumble, 1.0);

    // Setup the default command for the arm.
    arm.setDefaultCommand(new RunCommand(() -> arm.runAutomatic(), arm));
    // arm.setDefaultCommand(new RunCommand(() -> arm.dummy(), arm));

    // Configure the button bindings
    configureButtonBindings();

    // Configure autonomous chooser and Shuffleboard
    autoChooser = new SendableChooser<Command>();
    configureAutoChooser();
    walterTab = Shuffleboard.getTab("Walter");
    armPos = walterTab.add("Arm Position", -7.7).withPosition(4, 0).withSize(2, 2).getEntry();
    armLimit = walterTab.add("Arm Limit Switch", false).withPosition(2, 0).withSize(2, 2).getEntry();
    gripperLimit = walterTab.add("Gripper Limit Switch", false).withPosition(0, 0).withSize(2, 2).getEntry();
  }

  /** Define all button() to command() mappings. */
  private void configureButtonBindings() {

    // ########################################################
    // ################# DRIVER OI CONTROLLER #################
    // ########################################################

    // DRIVER Left & Right Stick: Translational and rotational robot movement.
    swerveDrive.setDefaultCommand(
        new RunCommand(
            () -> swerveDrive.drive(
                -MathUtil.applyDeadband(driverOI.getLeftY(), 0.05),
                -MathUtil.applyDeadband(driverOI.getLeftX(), 0.05),
                -MathUtil.applyDeadband(driverOI.getRightX(), 0.05),
                true),
            swerveDrive));

    // DRIVER Left Trigger: While held, switch to virtual high gear.
    driverOI.leftTrigger(0.25)
        .onTrue(new InstantCommand(() -> swerveDrive.setVirtualHighGear()))
        .onFalse(new InstantCommand(() -> swerveDrive.setVirtualLowGear()));

    // DRIVER Right Trigger: While held, deploy intake to obtain a Cube.
    driverOI.rightTrigger()
        .whileTrue(new Deploy(intake, uprighter, gripper, driverRumbler))
        .onFalse(new Retract(intake, uprighter, gripper));

    // DRIVER Left Bumper: Close intake flappers.
    driverOI.leftBumper().onTrue(new InstantCommand(() -> intake.close(), intake));

    // DRIVER Right Bumper: Open intake flappers.
    driverOI.rightBumper().onTrue(new InstantCommand(() -> intake.open(), intake));

    // DRIVER POV/D-Pad: Nudge (Left, Right, Up, Down) relative to the robot.
    driverOI.povUp().whileTrue(new Nudge(swerveDrive, "DRIVER NUDGE FORWARD", false).withTimeout(0.5));
    driverOI.povDown().whileTrue(new Nudge(swerveDrive, "DRIVER NUDGE BACKWARD", false).withTimeout(0.5));
    driverOI.povLeft().whileTrue(new Nudge(swerveDrive, "DRIVER NUDGE LEFT", false).withTimeout(0.5));
    driverOI.povRight().whileTrue(new Nudge(swerveDrive, "DRIVER NUDGE RIGHT", false).withTimeout(0.5));

    // DRIVER X,Y,B,A Buttons: Set chassis to predefined field relative angle.
    driverOI.x().whileTrue(new Rotate(swerveDrive, (double) 90));
    driverOI.y().whileTrue(new Rotate(swerveDrive, (double) 0));
    driverOI.b().whileTrue(new Rotate(swerveDrive, (double) -90));
    driverOI.a().whileTrue(new Rotate(swerveDrive, (double) 180));

    // DRIVER Start Button: Reset gyro to a new field oriented forward position.
    driverOI.start().whileTrue(new InstantCommand(() -> swerveDrive.resetGyro(), swerveDrive));

    // DRIVER Back Button: Set swerve drive to a stationary X position.
    driverOI.back().whileTrue(new RunCommand(() -> swerveDrive.setWheelsToXFormation(), swerveDrive));

    // ########################################################
    // ################ OPERATOR OI CONTROLLER ################
    // ########################################################

    // OPERATOR Left Stick: Spin gripper motors.
    gripper.setDefaultCommand(
        new RunCommand(() -> gripper.spin(-MathUtil.applyDeadband(operatorOI.getLeftY(), 0.05)), gripper));

    // OPERATOR Right Stick: Direct control over the Arm.
    new Trigger(() -> Math.abs(operatorOI.getRightY()) > 0.05)
        .whileTrue((new RunCommand(() -> arm.runManual(-operatorOI.getRightY()), arm)));

    // OPERATOR Left Trigger: While held, lower the intake.
    operatorOI.leftTrigger().onTrue(new InstantCommand(() -> intake.lower(), intake))
        .onFalse(new InstantCommand(() -> intake.raise(), intake));

    // OPERATOR Right Trigger: Outtake a Cube (intake, uprighter, gripper).
    operatorOI.rightTrigger().whileTrue(new Outtake(intake, uprighter, gripper))
        .onFalse(new Retract(intake, uprighter, gripper));

    // OPERATOR Left Bumper: Close gripper.
    // operatorOI.leftBumper().onTrue(new InstantCommand(() -> gripper.close(),
    // gripper));
    operatorOI.leftBumper().whileTrue(new Close(gripper, driverRumbler, operatorRumbler));

    // OPERATOR Right Bumper: Open gripper.
    operatorOI.rightBumper().onTrue(new InstantCommand(() -> gripper.open(), gripper));

    // OPERATOR X, Y, B: Move arm to preset positions.
    operatorOI.x().onTrue(new InstantCommand(() -> arm.setTargetPosition(POSITION_00)));
    operatorOI.y().onTrue(new InstantCommand(() -> arm.setTargetPosition(POSITION_01)));
    operatorOI.b().onTrue(new InstantCommand(() -> arm.setTargetPosition(POSITION_02)));

    // OPERATOR A: Move arm to preset positions. (EXPERIMENTAL)
    operatorOI.a().onTrue(new InstantCommand(() -> arm.setTargetPosition(POSITION_01)) // will it move off of 0.0 ???
        .andThen(new InstantCommand(() -> arm.setTargetPosition(POSITION_02)))
        .andThen(new InstantCommand(() -> arm.setTargetPosition(POSITION_03)))
        .andThen(new InstantCommand(() -> arm.setTargetPosition(POSITION_04)))
        .andThen(new InstantCommand(() -> arm.setTargetPosition(POSITION_05)))
        .andThen(new InstantCommand(() -> arm.setTargetPosition(POSITION_06))));

    operatorOI.start().onTrue(new ShootCube(gripper, intake, uprighter).withTimeout(1.0));

    // OPERATOR POV/D-Pad: Nudge (Left, Right, Up, Down) relative to the field.
    operatorOI.povUp().whileTrue(new Nudge(swerveDrive, "OPERATOR NUDGE FORWARD", true).withTimeout(0.5));
    operatorOI.povDown().whileTrue(new Nudge(swerveDrive, "OPERATOR NUDGE BACKWARD", true).withTimeout(0.5));
    operatorOI.povLeft().whileTrue(new Nudge(swerveDrive, "OPERATOR NUDGE LEFT", true).withTimeout(0.5));
    operatorOI.povRight().whileTrue(new Nudge(swerveDrive, "OPERATOR NUDGE RIGHT", true).withTimeout(0.5));

  }

  public void configureAutoChooser() {
    autoChooser.setDefaultOption("Default Auto", null);
    autoChooser.addOption("Original Auto", new OriginalAuto(swerveDrive));
    autoChooser.addOption("Auto Test", new AutoTest(swerveDrive, intake, uprighter, gripper, arm));
    SmartDashboard.putData(RobotContainer.autoChooser);
  }

  /**
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
