package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.CANID;
import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {

  private CANSparkMax intakeMotorLeft;
  private CANSparkMax intakeMotorRight;
  private Solenoid intakePistons;
  private Solenoid lifterPistons;

  public Intake() {

    // Intake Motor Left
    intakeMotorLeft = new CANSparkMax(CANID.LEFT_INTAKE_SPARKMAX, MotorType.kBrushless);
    intakeMotorLeft.restoreFactoryDefaults();
    intakeMotorLeft.setIdleMode(IdleMode.kBrake);
    intakeMotorLeft.setInverted(false);

    // Intake Motor Right
    intakeMotorRight = new CANSparkMax(CANID.RIGHT_INTAKE_SPARKMAX, MotorType.kBrushless);
    intakeMotorRight.restoreFactoryDefaults();
    intakeMotorRight.setIdleMode(IdleMode.kBrake);
    intakeMotorRight.setInverted(false);

    // Intake Pistons
    intakePistons = new Solenoid(PneumaticsModuleType.CTREPCM, INTAKE_PISTON_SOLENOID_CHANNEL);

    // Lifter Pistons
    lifterPistons = new Solenoid(PneumaticsModuleType.CTREPCM, LIFTER_PISTON_SOLENOID_CHANNEL);

    // Set the starting position of the intake subsystem.
    open();
    raise();
  }

  @Override
  public void periodic() {
  }

  // Intake Motors

  public void spinIn() {
    intakeMotorLeft.set(INTAKE_SPEED);
    intakeMotorRight.set(-INTAKE_SPEED);
  }

  public void spinOut() {
    intakeMotorLeft.set(-OUTTAKE_SPEED);
    intakeMotorRight.set(OUTTAKE_SPEED);
  }

  public void stop() {
    intakeMotorLeft.stopMotor();
    intakeMotorRight.stopMotor();
  }

  // Intake Pistons

  public void close() {
    intakePistons.set(true);
  }

  public void open() {
    intakePistons.set(false);
  }

  // Lifter Pistons

  public void raise() {
    lifterPistons.set(false);
  }

  public void lower() {
    lifterPistons.set(true);
  }
}
