package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.autonomous.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.ArmConstants.*;

public class RobotContainer {

  // Create subsystems
  private final SwerveDrive swerveDrive;
  private final Gripper gripper;
  private final Arm arm;
  private final Intake intake;
  private final Uprighter uprighter;
  Compressor compressor;

  // OI controllers
  CommandXboxController driverOI;
  CommandXboxController operatorOI;

  // Autonomous Commands
  private final DefaultAuto defaultAuto;
  private final PickupObject pickupObject;

  // NetworkTables
  public static NetworkTableInstance networkTableInstance;
  public static NetworkTable gamePieceIDs; // ???
  public static NetworkTable gamePieceCoordinates;

  // Used for determining if gripper is picking up cone or cube
  public static boolean readyForCone = false;

  /** Robot Container Constructor. */
  public RobotContainer() {

    // Instantiate all subsystems
    swerveDrive = new SwerveDrive();
    gripper = new Gripper();
    arm = new Arm();
    intake = new Intake();
    uprighter = new Uprighter();
    compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    // Instantiate all autonomous commands
    defaultAuto = new DefaultAuto(swerveDrive);
    pickupObject = new PickupObject(swerveDrive, intake, uprighter, gripper);

    // Instantiate all OI controllers
    driverOI = new CommandXboxController(0);
    operatorOI = new CommandXboxController(1);

    // Custom NetworkTables
    networkTableInstance = NetworkTableInstance.getDefault();
    gamePieceIDs = networkTableInstance.getTable("Piece"); // ???
    gamePieceCoordinates = networkTableInstance.getTable("Vision");

    // Configure the button bindings
    configureButtonBindings();

    SmartDashboard.putNumber("Setpoint", arm.getEncoderVal());
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
                -MathUtil.applyDeadband(driverOI.getLeftY(), 0.07),
                -MathUtil.applyDeadband(driverOI.getLeftX(), 0.07),
                -MathUtil.applyDeadband(driverOI.getRightX(), 0.07),
                true),
            swerveDrive));

    // DRIVER Left Trigger: (WH) Switch to virtual high gear.
    driverOI.leftTrigger(0.25)
        .onTrue(new InstantCommand(() -> swerveDrive.setVirtualHighGear()))
        .onFalse(new InstantCommand(() -> swerveDrive.setVirtualLowGear()));

    // DRIVER Right Trigger: (WH) Deploy intake when pressed and spin motors.
    driverOI.rightTrigger().whileTrue(new DeployIntake(intake, uprighter, gripper));

    // DRIVER Right Trigger: (WR) Spin intake motors, close flappers, lifter up.
    driverOI.rightTrigger().onFalse(new RetractIntake(intake, uprighter, gripper).withTimeout(1));
    driverOI.rightBumper().onTrue(new InstantCommand(() -> gripper.gripPiece()));

    // DRIVER POV/D-Pad: Nudge (Left, Right, Up, Down) relative to the robot.
    driverOI.povRight().onTrue(new InstantCommand(() -> swerveDrive.drive(0.0, -0.5, 0, false)).withTimeout(0.25));
    driverOI.povLeft().onTrue(new InstantCommand(() -> swerveDrive.drive(0.0, 0.5, 0, false)).withTimeout(0.25));
    driverOI.povUp().onTrue(new InstantCommand(() -> swerveDrive.drive(0.5, 0.0, 0, false)).withTimeout(0.25));
    driverOI.povDown().onTrue(new InstantCommand(() -> swerveDrive.drive(-0.5, 0.0, 0, false)).withTimeout(0.25));

    // DRIVER Start Button: Reset the robot's field oriented forward position.
    driverOI.start().whileTrue(new RunCommand(() -> swerveDrive.resetFieldOrientedGyro(), swerveDrive));

    // DRIVER Back Button: While held, defensive X position and prevent driving.
    operatorOI.back().whileTrue(new RunCommand(() -> swerveDrive.setX(), swerveDrive));

    // ########################################################
    // ################ OPERATOR OI CONTROLLER ################
    // ########################################################

    // OPERATOR Left Stick: Direct control over the Arm.
    arm.setDefaultCommand(new RunCommand(() -> arm.runAutomatic(operatorOI.getLeftY()), arm));

    // OPERATOR Right Stick: Direct control over the Uprighter.
    uprighter.setDefaultCommand(new RunCommand(
        () -> uprighter.spin(-MathUtil.applyDeadband(operatorOI.getRightY(), 0.07)), uprighter));

    // OPERATOR Left Trigger: While held, auto pick up game object using ML/AI.
    operatorOI.leftTrigger().onTrue(pickupObject.runAutoSequence());

    // OPERATOR Right Trigger: Release game object from Grabber.
    operatorOI.rightTrigger().whileTrue(new OuttakeGamePiece(gripper));

    // OPERATOR POV/D-Pad: Nudge (Left, Right, Up, Down) relative to the field.
    operatorOI.povRight().onTrue(new InstantCommand(() -> swerveDrive.drive(-0.5, 0.0, 0, true)).withTimeout(0.25));
    operatorOI.povLeft().onTrue(new InstantCommand(() -> swerveDrive.drive(0.5, 0.0, 0, true)).withTimeout(0.25));
    operatorOI.povUp().onTrue(new InstantCommand(() -> swerveDrive.drive(0.0, 0.5, 0, true)).withTimeout(0.25));
    operatorOI.povDown().onTrue(new InstantCommand(() -> swerveDrive.drive(0.0, -0.5, 0, true)).withTimeout(0.25));

    // OPERATOR A, B, X, Y: Set the arm to a specific predefined position.
    operatorOI.a().onTrue(new InstantCommand(() -> arm.setTargetPosition(HOME_POSITION)));
    operatorOI.b().onTrue(new InstantCommand(() -> arm.setTargetPosition(SCORING_POSITION)));
    operatorOI.x().onTrue(new InstantCommand(() -> arm.setTargetPosition(INTAKE_POSITION)));
    operatorOI.y().onTrue(new InstantCommand(() -> arm.setTargetPosition(FEEDER_POSITION)));

    // Button: Cycle out everything (intake, uprighter, gripper).
    operatorOI.start().whileTrue(new SpinEverythingOut(intake, uprighter, gripper));

    // OPERATOR Back Button: Toggle the gripper state. (Cube/Cone)
    operatorOI.back().onTrue(new InstantCommand(() -> gripper.setGripperState()));

  }

  /**
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return defaultAuto.runAutoSequence();
  }

}
