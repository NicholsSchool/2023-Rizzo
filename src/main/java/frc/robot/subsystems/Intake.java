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
    intakeMotorRight.setInverted(true);

    // Intake Pistons
    intakePistons = new Solenoid(PneumaticsModuleType.CTREPCM, INTAKE_PISTON_SOLENOID_CHANNEL);
    intakePistons.set(INTAKE_CLOSED);

    // Lifter Pistons
    lifterPistons = new Solenoid(PneumaticsModuleType.CTREPCM, LIFTER_PISTON_SOLENOID_CHANNEL);
    lifterPistons.set(LIFTER_UP);
  }

  @Override
  public void periodic() {
  }

  // Intake Motors

  public void spinIn() {
    intakeMotorLeft.set(INTAKE_SPEED);
    intakeMotorRight.set(INTAKE_SPEED);
  }

  public void spinOut() {
    intakeMotorLeft.set(-INTAKE_SPEED);
    intakeMotorRight.set(-INTAKE_SPEED);
  }

  public void stop() {
    intakeMotorLeft.stopMotor();
    intakeMotorRight.stopMotor();
  }

  // Intake Pistons

  public void flapperClose() {
    intakePistons.set(INTAKE_CLOSED);
  }

  public void flapperOpen() {
    intakePistons.set(!INTAKE_CLOSED);
  }

  // Lifter Pistons

  public void lifterUp() {
    lifterPistons.set(LIFTER_UP);
  }

  public void lifterDown() {
    lifterPistons.set(!LIFTER_UP);
  }
}
