package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {

  // Subsystems
  public final SwerveDrive swerveDrive;

  // OI (Operator Interface) controllers
  public static CommandXboxController driverOI;
  public static CommandXboxController operatorOI;
  public static XboxController driverRumbler;
  public static XboxController operatorRumbler;

  // Autonomous Chooser
  public static SendableChooser<Command> autoChooser;

  public RobotContainer() {

    // Instantiate all subsystems
    swerveDrive = new SwerveDrive();

    // OI (Operator Interface) Controllers & Rumblers
    driverOI = new CommandXboxController(1);
    driverRumbler = new XboxController(1);
    operatorOI = new CommandXboxController(0);
    operatorRumbler = new XboxController(0);

    // Configure the button bindings
    configureButtonBindings();

    // Configure autonomous chooser
    autoChooser = new SendableChooser<Command>();
    configureAutoChooser();
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

    // OPERATOR Start Button: Reset max Pitch/Roll on the dashboard.
    operatorOI.start().whileTrue(new InstantCommand(() -> swerveDrive.resetMaxPitchRoll()));

  }

  /**
   * Configure the autonomous chooser.
   */
  public void configureAutoChooser() {
    SmartDashboard.putData(RobotContainer.autoChooser);
  }

  /**
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
