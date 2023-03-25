package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import static frc.robot.Constants.ArmConstants.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {

  // Subsystems
  private final SwerveDrive swerveDrive;
  private final Gripper gripper;
  private final Arm arm;
  private final Intake intake;
  private final Uprighter uprighter;

  // OI controllers
  CommandXboxController driverOI;
  CommandXboxController operatorOI;
  XboxController driverRumbler;
  XboxController operatorRumbler;

  // Autonomous Chooser
  SendableChooser<Command> autoChooser;

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

    // Configure the button bindings
    configureButtonBindings();

    // Configure autonomous chooser and add to SmartDashboard
    autoChooser = new SendableChooser<>();
    configureAutoChooser(autoChooser);

  }

  /** Define all button() to command() mappings. */
  private void configureButtonBindings() {

    // ########################################################
    // ################# DRIVER OI CONTROLLER #################
    // ########################################################

    // DRIVER Left & Right Stick: Field relative translational/rotational movement.
    swerveDrive.setDefaultCommand(
        new RunCommand(
            () -> swerveDrive.drive(
                -MathUtil.applyDeadband(driverOI.getLeftY(), 0.05),
                -MathUtil.applyDeadband(driverOI.getLeftX(), 0.05),
                -MathUtil.applyDeadband(driverOI.getRightX(), 0.05),
                true),
            swerveDrive));

    // DRIVER Left Trigger: (WH) Switch to virtual high gear.
    driverOI.leftTrigger(0.25)
        .onTrue(new InstantCommand(() -> swerveDrive.setVirtualHighGear()))
        .onFalse(new InstantCommand(() -> swerveDrive.setVirtualLowGear()));

    // DRIVER Right Trigger: (WH) Deploy intake when pressed and spin motors in.
    driverOI.rightTrigger().whileTrue(new IntakeDeploy(intake, uprighter, gripper));
    driverOI.rightTrigger().onFalse(new IntakeRetract(intake, uprighter, gripper));

    // DRIVER POV/D-Pad: Nudge (Left, Right, Up, Down) relative to the robot.
    driverOI.povLeft().whileTrue(new RunCommand(() -> swerveDrive.drive(0.0, 0.5, 0, true)));
    driverOI.povRight().whileTrue(new RunCommand(() -> swerveDrive.drive(0.0, -0.5, 0, true)));
    driverOI.povUp().whileTrue(new RunCommand(() -> swerveDrive.drive(0.5, 0.0, 0, true)));
    driverOI.povDown().whileTrue(new RunCommand(() -> swerveDrive.drive(-0.5, 0.0, 0, true)));

    // DRIVER Start Button: Reset the robot's field oriented forward position.
    driverOI.start().whileTrue(new RunCommand(() -> swerveDrive.resetFieldOrientedGyro(), swerveDrive));

    // DRIVER Back Button: While held, defensive X position and prevent driving.
    driverOI.x().whileTrue(new RunCommand(() -> swerveDrive.setX(), swerveDrive));

    // ########################################################
    // ################ OPERATOR OI CONTROLLER ################
    // ########################################################

    // OPERATOR Left Stick: Direct control over the Arm.
    new Trigger(() -> Math.abs(operatorOI.getLeftY()) > 0.05)
        .whileTrue((new RunCommand(() -> arm.runManual(-operatorOI.getLeftY()), arm)));

    // OPERATOR X, Y, B, A: Move arm to preset positions.
    arm.setDefaultCommand(new RunCommand(() -> arm.runAutomatic(), arm));
    operatorOI.x().onTrue(new InstantCommand(() -> arm.setTargetPosition(HOME_POSITION)));
    operatorOI.y().onTrue(new InstantCommand(() -> arm.setTargetPosition(HUMAN_PLAYER_POSITION)));
    operatorOI.b().onTrue(new InstantCommand(() -> arm.setTargetPosition(SCORING_POSITION)));
    operatorOI.a().onTrue(new InstantCommand(() -> arm.setTargetPosition(GROUND_POSITION)));

    // OPERATOR Right Stick: Direct control over the Gripper motors.
    uprighter.setDefaultCommand(
        new RunCommand(() -> gripper.spin(-MathUtil.applyDeadband(operatorOI.getLeftY(), 0.05)), uprighter));

    // OPERATOR Right Trigger: Release game object from Grabber.
    operatorOI.rightTrigger().whileTrue(new IntakeExtract(intake, uprighter, gripper));
    operatorOI.rightTrigger().onFalse(new IntakeRetract(intake, uprighter, gripper));

    // OPERATOR POV/D-Pad: Nudge (Left, Right, Up, Down) relative to the field.
    operatorOI.povUp().whileTrue(new RunCommand(() -> swerveDrive.drive(-0.5, 0.0, 0, true)));
    operatorOI.povDown().whileTrue(new RunCommand(() -> swerveDrive.drive(0.5, 0.0, 0, true)));
    operatorOI.povRight().whileTrue(new RunCommand(() -> swerveDrive.drive(0.0, 0.5, 0, true)));
    operatorOI.povLeft().whileTrue(new RunCommand(() -> swerveDrive.drive(0.0, -0.5, 0, true)));

  }

  public void configureAutoChooser(SendableChooser<Command> autoChooser) {
    autoChooser.setDefaultOption("Default Auto", null);
    autoChooser.addOption("Swerve Auto", new DefaultAuto(swerveDrive));
    autoChooser.addOption("Auto Test Class", new AutoTest(swerveDrive, intake, uprighter, gripper));
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
