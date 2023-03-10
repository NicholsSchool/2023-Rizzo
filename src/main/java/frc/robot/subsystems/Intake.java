package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.CANID;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private CANSparkMax intakeLeft;
  private CANSparkMax intakeRight;
  private Solenoid lifterLeft;
  private Solenoid lifterRight;

  public Intake() {

    // Motor
    intakeLeft = new CANSparkMax(CANID.LEFT_INTAKE_SPARKMAX, MotorType.kBrushless);
    intakeRight = new CANSparkMax(CANID.RIGHT_INTAKE_SPARKMAX, MotorType.kBrushless);

    intakeLeft.restoreFactoryDefaults();
    intakeRight.restoreFactoryDefaults();

    intakeLeft.setIdleMode(IdleMode.kBrake);
    intakeRight.setIdleMode(IdleMode.kBrake);

    intakeLeft.setInverted(false);
    intakeRight.setInverted(false); // experiment with inverting using this

    // Lifter
    lifterLeft = new Solenoid(PneumaticsModuleType.CTREPCM, IntakeConstants.LIFTER_LEFT_SOLENOID_CHANNEL);
    lifterRight = new Solenoid(PneumaticsModuleType.CTREPCM, IntakeConstants.LIFTER_RIGHT_SOLENOID_CHANNEL);

    lifterLeft.set(!IntakeConstants.INTAKE_PISTON_EXTENDED);
    lifterRight.set(!IntakeConstants.INTAKE_PISTON_EXTENDED);

  }

  public void in(double speed) {
    intakeLeft.set(speed);
    intakeRight.set(speed);
  }

  public void out(double speed) {
    intakeLeft.set(-speed);
    intakeRight.set(-speed);
  }

  public void stop() {
    intakeLeft.stopMotor();
    intakeRight.stopMotor();
  }

  public void raise() {
    if (lifterLeft.get() == IntakeConstants.INTAKE_PISTON_EXTENDED) {
      lifterLeft.set(!IntakeConstants.INTAKE_PISTON_EXTENDED);
      lifterRight.set(!IntakeConstants.INTAKE_PISTON_EXTENDED);
    }
  }

  public void lower() {
    if (lifterLeft.get() != IntakeConstants.INTAKE_PISTON_EXTENDED) {
      lifterLeft.set(IntakeConstants.INTAKE_PISTON_EXTENDED);
      lifterRight.set(IntakeConstants.INTAKE_PISTON_EXTENDED);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
