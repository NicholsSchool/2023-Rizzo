package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import static frc.robot.Constants.ArmConstants.*;

public class RobotContainer {

  // Subsystems
  public final SwerveDrive swerveDrive;
  public final Hand hand;
  public final Arm arm;
  public final Wrist wrist;

  // OI (Operator Interface) controllers
  public static CommandXboxController driverOI;
  public static CommandXboxController operatorOI;
  public static XboxController driverRumbler;
  public static XboxController operatorRumbler;

  public RobotContainer() {

    // Instantiate all subsystems
    swerveDrive = new SwerveDrive();
    hand = new Hand();
    arm = new Arm();
    wrist = new Wrist();

    // OI (Operator Interface) Controllers & Rumblers
    driverOI = new CommandXboxController(1);
    driverRumbler = new XboxController(1);
    operatorOI = new CommandXboxController(0);
    operatorRumbler = new XboxController(0);

    // Configure the button bindings
    configureButtonBindings();
  }

  /** Define all button() to command() mappings. */
  private void configureButtonBindings() {

    // DRIVER Left & Right Stick: Translational and rotational robot movement.
    swerveDrive.setDefaultCommand(
        new RunCommand(
            () -> swerveDrive.drive(
                -MathUtil.applyDeadband(driverOI.getLeftY() * 0.8, 0.05),
                -MathUtil.applyDeadband(driverOI.getLeftX() * 0.8, 0.05),
                -MathUtil.applyDeadband(driverOI.getRightX() * 0.4, 0.05),
                true),
            swerveDrive));

    // DRIVER Left Trigger: While held, switch to virtual high gear.
    driverOI.leftTrigger()
        .onTrue(new InstantCommand(() -> swerveDrive.setVirtualHighGear()))
        .onFalse(new InstantCommand(() -> swerveDrive.setVirtualLowGear()));

    // DRIVER Right Trigger: While held, intake the hand
    driverOI.rightTrigger().whileTrue(new HandIntake(hand));

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

    // Operator Right Trigger: While held, shoot from hand
    operatorOI.leftTrigger().whileTrue(new HandShoot(hand));

    // Operator Right Bumper: While held, outtake from hand
    operatorOI.leftTrigger().whileTrue(new HandOuttake(hand));

    // Operator B: Toggle wrist position
    operatorOI.b().onTrue(new WristToggle(wrist));

    // OPERATOR Left Stick Y: Direct control over the Arm.
    new Trigger(() -> Math.abs(operatorOI.getLeftY()) > 0.05)
        .whileTrue(new ArmDirectControl(arm, -operatorOI.getLeftY()));

    // Operator DPAD Up, Right, Left, Down: Set Arm Target Positions
    operatorOI.povUp().onTrue(new ArmToAngle(arm, ARM_HIGH_POS));
    operatorOI.povLeft().onTrue(new ArmToAngle(arm, ARM_MED_POS));
    operatorOI.povRight().onTrue(new ArmToAngle(arm, ARM_MED_POS));
    operatorOI.povDown().onTrue(new ArmToAngle(arm, ARM_LOW_POS));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}