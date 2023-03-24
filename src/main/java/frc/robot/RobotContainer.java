package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import java.util.List;

import frc.robot.Constants.SwerveDriveConstants;
import static frc.robot.Constants.ArmConstants.*;

public class RobotContainer {

  // Create subsystems
  private final SwerveDrive swerveDrive;
  private final Gripper gripper;
  // private final Arm arm;
  private final CBArm cbarm;
  private final Intake intake;
  private final Uprighter uprighter;
  Compressor compressor;

  // OI controllers
  CommandXboxController driverOI;
  CommandXboxController operatorOI;

  // Autonomous Commands
  // private final DefaultAuto defaultAuto;

  // Used for determining if gripper is picking up cone or cube
  public static boolean readyForCube = false;

  /** Robot Container Constructor. */
  public RobotContainer() {

    // Instantiate all subsystems
    swerveDrive = new SwerveDrive();
    gripper = new Gripper();
    // arm = new Arm();
    cbarm = new CBArm();
    intake = new Intake();
    uprighter = new Uprighter();
    compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    // Instantiate all autonomous commands
    // defaultAuto = new DefaultAuto(swerveDrive);

    // Instantiate all OI controllers
    driverOI = new CommandXboxController(1);
    operatorOI = new CommandXboxController(0);

    // Configure the button bindings
    configureButtonBindings();
  }

  /** Define all button() to command() mappings. */
  private void configureButtonBindings() {

    // ########################################################
    // ################# DRIVER OI CONTROLLER #################
    // ########################################################

    // DRIVER Left & Right Stick: Field relative translational/rotational movement.
    // working
    swerveDrive.setDefaultCommand(
        new RunCommand(
            () -> swerveDrive.drive(
                -MathUtil.applyDeadband(driverOI.getLeftY(), 0.07),
                -MathUtil.applyDeadband(driverOI.getLeftX(), 0.07),
                -MathUtil.applyDeadband(driverOI.getRightX(), 0.07),
                true),
            swerveDrive));

    // DRIVER Left Trigger: (WH) Switch to virtual high gear.
    // working
    driverOI.leftTrigger(0.25)
        .onTrue(new InstantCommand(() -> swerveDrive.setVirtualHighGear()))
        .onFalse(new InstantCommand(() -> swerveDrive.setVirtualLowGear()));

    // DRIVER Right Trigger: (WH) Deploy intake when pressed and spin motors.
    // working
    driverOI.rightTrigger().whileTrue(new IntakeDeploy(intake, uprighter, gripper));
    driverOI.rightTrigger().onFalse(new IntakeRetract(intake, uprighter, gripper));
  

    // DRIVER POV/D-Pad: Nudge (Left, Right, Up, Down) relative to the robot.
    // working
    driverOI.povLeft().whileTrue( new RunCommand(() -> swerveDrive.drive(0.0, 0.5, 0, true)));
    driverOI.povRight().whileTrue(new RunCommand(() -> swerveDrive.drive(0.0, -0.5, 0, true)));
    driverOI.povUp().whileTrue(new RunCommand(() -> swerveDrive.drive(0.5, 0.0, 0, true)));
    driverOI.povDown().whileTrue(new RunCommand(() -> swerveDrive.drive(-0.5, 0.0, 0, true)));

    // DRIVER Start Button: Reset the robot's field oriented forward position.
    // working
    driverOI.start().whileTrue(new RunCommand(() -> swerveDrive.resetFieldOrientedGyro(), swerveDrive));
  

    // DRIVER Back Button: While held, defensive X position and prevent driving.
    // NOT working (Camden)
    driverOI.x().whileTrue(new RunCommand(() -> swerveDrive.setX(), swerveDrive));

    // ########################################################
    // ################ OPERATOR OI CONTROLLER ################
    // ########################################################

    // OPERATOR Left Stick: Direct control over the Arm.
    cbarm.setDefaultCommand(new RunCommand(() -> cbarm.move(-operatorOI.getLeftY()), cbarm));

    operatorOI.x().onTrue(new GoToPos(HOME_POSITION, cbarm));
    // operatorOI.y().onTrue(new InstantCommand(() ->
    // cbarm.setPositionUsingPID(HUMAN_PLAYER_POSITION)));
    operatorOI.y().whileTrue(new GoToPos(20, cbarm));
    // operatorOI.b().onTrue(new InstantCommand(() ->
    // cbarm.setPositionUsingPID(SCORING_POSITION)));
    // operatorOI.a().onTrue(new InstantCommand(() ->
    // cbarm.setPositionUsingPID(GROUND_POSITION)));

    // OPERATOR Right Stick: Direct control over the Uprighter.
    // working
    uprighter.setDefaultCommand(new RunCommand(
        () -> uprighter.spin(-MathUtil.applyDeadband(operatorOI.getRightY(), 0.07)), uprighter));

    // OPERATOR Right Trigger: Release game object from Grabber.
    // working
    operatorOI.rightTrigger().whileTrue(new IntakeExtract(intake, uprighter, gripper));
    operatorOI.rightTrigger().onFalse(new IntakeRetract(intake, uprighter, gripper));

    // OPERATOR POV/D-Pad: Nudge (Left, Right, Up, Down) relative to the field.
    // working
    operatorOI.povUp().whileTrue(new RunCommand(() -> swerveDrive.drive(-0.5, 0.0, 0, true)));
    operatorOI.povDown().whileTrue(new RunCommand(() -> swerveDrive.drive(0.5, 0.0, 0, true)));
    operatorOI.povRight().whileTrue(new RunCommand(() -> swerveDrive.drive(0.0, 0.5, 0, true)));
    operatorOI.povLeft().whileTrue(new RunCommand(() -> swerveDrive.drive(0.0, -0.5, 0, true)));

  }

  /**
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new AutoTest(swerveDrive, cbarm, intake, uprighter, gripper); 
  }

}
