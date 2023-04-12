package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.IntakeConstants.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.networktables.GenericEntry;

public class RobotContainer {

  // Subsystems
  public final SwerveDrive swerveDrive;
  public final Gripper gripper;
  public final Arm arm;
  public final Intake intake;
  public final Uprighter uprighter;

  // Shuffleboard
  ShuffleboardTab walterTab;
  public static GenericEntry armPos;
  public static GenericEntry leftArmLimit;
  public static GenericEntry rightArmLimit;
  public static GenericEntry gripperLimit;
  public static ComplexWidget autoChooserWidget;

  // OI (Operator Interface) controllers
  public static CommandXboxController driverOI;
  public static CommandXboxController operatorOI;
  public static XboxController driverRumbler;
  public static XboxController operatorRumbler;

  // Autonomous Chooser
  public static SendableChooser<Command> autoChooser;
  public Alliance alliance;

  public RobotContainer() {

    // Instantiate all subsystems
    swerveDrive = new SwerveDrive();
    intake = new Intake();
    gripper = new Gripper();
    uprighter = new Uprighter();
    arm = new Arm();

    // OI (Operator Interface) Controllers & Rumblers
    driverOI = new CommandXboxController(1);
    driverRumbler = new XboxController(1);
    operatorOI = new CommandXboxController(0);
    operatorRumbler = new XboxController(0);

    // Setup the default command for the arm.
    arm.setDefaultCommand(new RunCommand(() -> arm.runAutomatic(), arm));

    // Configure the button bindings
    configureButtonBindings();

    // Configure autonomous chooser and Shuffleboard
    autoChooser = new SendableChooser<Command>();
    configureAutoChooser();

    // Configure the Shuffleboard
    walterTab = Shuffleboard.getTab("Walter");
    armPos = walterTab.add("Arm Position", -7.7).withPosition(18, 0).withSize(6, 6).getEntry();
    leftArmLimit = walterTab.add("Arm L LS", false).withPosition(6, 0).withSize(6, 6).getEntry();
    rightArmLimit = walterTab.add("Arm R LS", false).withPosition(12, 0).withSize(6, 6).getEntry();
    gripperLimit = walterTab.add("Gripper LS", false).withPosition(0, 0).withSize(6, 6).getEntry();
  }

  /** Define all button() to command() mappings. */
  private void configureButtonBindings() {

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
    driverOI.leftTrigger()
        .onTrue(new InstantCommand(() -> swerveDrive.setVirtualHighGear()))
        .onFalse(new InstantCommand(() -> swerveDrive.setVirtualLowGear()));

    // DRIVER Right Trigger: While held, deploy intake to obtain a Cube.
    driverOI.rightTrigger().whileTrue(new DeployIntake(intake, uprighter));

    // DRIVER Right Bumper: While held, close intake flappers.
    driverOI.rightBumper()
        .onTrue(new InstantCommand(() -> intake.close(), intake))
        .onFalse(new InstantCommand(() -> intake.open(), intake));

    // DRIVER POV/D-Pad: Nudge (Left, Right, Up, Down) relative to the robot.
    driverOI.povUp().whileTrue(new NudgeRobot(swerveDrive, "NUDGE FORWARD").withTimeout(0.5));
    driverOI.povDown().whileTrue(new NudgeRobot(swerveDrive, "NUDGE BACKWARD").withTimeout(0.5));
    driverOI.povLeft().whileTrue(new NudgeRobot(swerveDrive, "NUDGE LEFT").withTimeout(0.5));
    driverOI.povRight().whileTrue(new NudgeRobot(swerveDrive, "NUDGE RIGHT").withTimeout(0.5));

    // DRIVER X,Y,B,A Buttons: Set chassis to predefined field relative angle.
    driverOI.x().whileTrue(new RotateRobot(swerveDrive, (double) -90));
    driverOI.y().whileTrue(new RotateRobot(swerveDrive, (double) 0));
    driverOI.b().whileTrue(new RotateRobot(swerveDrive, (double) 90));
    driverOI.a().whileTrue(new RotateRobot(swerveDrive, (double) 180));

    // DRIVER Back Button: Set swerve drive to a stationary X position.
    driverOI.back().whileTrue(new RunCommand(() -> swerveDrive.setWheelsToXFormation(), swerveDrive));

    // DRIVER Start Button: Reset gyro to a new field oriented forward position.
    driverOI.start().whileTrue(new InstantCommand(() -> swerveDrive.resetGyro(), swerveDrive));

    // OPERATOR Left Stick: Spin gripper motors and rumble.
    new Trigger(() -> Math.abs(operatorOI.getLeftY()) > 0.05)
        .whileTrue(new SpinGripper(driverRumbler, operatorRumbler, gripper));

    // OPERATOR Right Stick: Direct control over the Arm.
    new Trigger(() -> Math.abs(operatorOI.getRightY()) > 0.05)
        .whileTrue((new RunCommand(() -> arm.runManual(-operatorOI.getRightY()), arm)));

    // OPERATOR Left Trigger: While held, lower the intake.
    operatorOI.leftTrigger().onTrue(new InstantCommand(() -> intake.lower(), intake))
        .onFalse(new InstantCommand(() -> intake.raise(), intake));

    // OPERATOR Right Trigger: High power Outtake.
    operatorOI.rightTrigger().whileTrue(new OuttakeCube(intake, uprighter, gripper, OUTTAKE_HIGH_SPEED));

    // OPERATOR Left Bumper: While held, open the gripper.
    operatorOI.leftBumper().onTrue(new InstantCommand(() -> gripper.open(), gripper))
        .onFalse(new InstantCommand(() -> gripper.close(), gripper));

    // OPERATOR Right Bumper: Low power Outtake.
    operatorOI.rightBumper().whileTrue(new OuttakeCube(intake, uprighter, gripper, OUTTAKE_LOW_SPEED));

    // OPERATOR X,Y,B,A: Move arm to preset positions.
    operatorOI.x().onTrue(new InstantCommand(() -> arm.setTargetPosition(POSITION_00)));
    operatorOI.y().onTrue(new InstantCommand(() -> arm.setTargetPosition(POSITION_01)));
    operatorOI.b().onTrue(new InstantCommand(() -> arm.setTargetPosition(POSITION_02)));
    operatorOI.a().onTrue(new InstantCommand(() -> arm.setTargetPosition(POSITION_03)));

    // OPERATOR POV_DOWN: align to apriltag
    operatorOI.povDown().whileTrue(new ApriltagAlign(swerveDrive));

    // OPERATOR POV_UP: pick up cube using Machine Learning
    operatorOI.povUp().whileTrue(new MLPickup(swerveDrive).raceWith(new DeployIntake(intake, uprighter)));

    // OPERATOR Start Button: Reset max Pitch/Roll on the dashboard.
    operatorOI.start().whileTrue(new InstantCommand(() -> swerveDrive.resetMaxPitchRoll()));

  }

  public void configureAutoChooser() {
    autoChooser.setDefaultOption("Default Auto", new PrintCommand("Default Auto: Do Nothing"));
    autoChooser.addOption("Electric Red",
        new ElectricRed(swerveDrive, intake, uprighter, gripper, arm));
    autoChooser.addOption("Electric Blue",
        new ElectricBlue(swerveDrive, intake, uprighter, gripper, arm));
    autoChooser.addOption("Electric PP",
        new ElectricPathPlanner(swerveDrive, intake, uprighter, gripper, arm));
    autoChooser.addOption("Mayhem Red",
        new MayhemRed(swerveDrive, intake, uprighter, gripper, arm));
    autoChooser.addOption("Mayhem Blue",
        new MayhemBlue(swerveDrive, intake, uprighter, gripper, arm));
    autoChooser.addOption("Test 07 (Daniel's 3B Auto)",
        new Test07PathPlanner(swerveDrive, intake, uprighter, gripper, arm));
    autoChooser.addOption("Test 08 (Shoot, Mobility, Balance)",
        new Test08PathPlanner(swerveDrive, intake, uprighter, gripper, arm));
    autoChooser.addOption("Balance",
        new Balance(swerveDrive, intake, uprighter, gripper, arm));
    autoChooser.addOption("Two Cube Mayhem",
        new DoubleCubeAuto(swerveDrive, intake, uprighter, gripper, arm));
    autoChooser.addOption("Test 10 (Balance Forwards)",
        new Test10PathPlanner(swerveDrive, intake, uprighter, gripper, arm));
    SmartDashboard.putData(RobotContainer.autoChooser);
  }

  /**
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
