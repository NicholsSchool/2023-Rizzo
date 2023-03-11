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
  private CANSparkMax uprighterMotorLeft;
  private CANSparkMax uprighterMotorRight;

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

    // Uprighter Motor Left
    uprighterMotorLeft = new CANSparkMax(CANID.LEFT_UPRIGHTER_SPARKMAX, MotorType.kBrushless);
    uprighterMotorLeft.restoreFactoryDefaults();
    uprighterMotorLeft.setIdleMode(IdleMode.kBrake);
    uprighterMotorLeft.setInverted(false);

    // Uprighter Motor Right
    uprighterMotorRight = new CANSparkMax(CANID.RIGHT_UPRIGHTER_SPARKMAX, MotorType.kBrushless);
    uprighterMotorRight.restoreFactoryDefaults();
    uprighterMotorRight.setIdleMode(IdleMode.kBrake);
    uprighterMotorRight.setInverted(true);

  }

  @Override
  public void periodic() {
  }

  // Intake Motors

  public void intakeSpinIn() {
    intakeMotorLeft.set(INTAKE_SPEED);
    intakeMotorRight.set(INTAKE_SPEED);
  }

  public void intakeSpinOut() {
    intakeMotorLeft.set(-INTAKE_SPEED);
    intakeMotorRight.set(-INTAKE_SPEED);
  }

  public void intakeStop() {
    intakeMotorLeft.stopMotor();
    intakeMotorRight.stopMotor();
  }

  // Intake Pistons

  public void intakeClose() {
    intakePistons.set(INTAKE_CLOSED);
  }

  public void intakeOpen() {
    intakePistons.set(!INTAKE_CLOSED);
  }

  // Lifter Pistons

  public void lifterUp() {
    lifterPistons.set(LIFTER_UP);
  }

  public void lifterDown() {
    lifterPistons.set(!LIFTER_UP);
  }

  // Uprighter Motors

  public void uprighterSpinIn() {
    uprighterMotorLeft.set(-UPRIGHTER_SPEED);
    uprighterMotorRight.set(-UPRIGHTER_SPEED);
  }

  public void uprighterSpinOut() {
    uprighterMotorLeft.set(UPRIGHTER_SPEED);
    uprighterMotorRight.set(UPRIGHTER_SPEED);
  }

  public void uprighterStop() {
    uprighterMotorLeft.stopMotor();
    uprighterMotorRight.stopMotor();
  }

  /**
   * Spin the uprighter motors at a given speed.
   * 
   * @param speed double between -1.0 and 1.0
   */
  public void uprighterSpin(double speed) {
    uprighterMotorLeft.set(speed);
    uprighterMotorRight.set(speed);
  }

}
