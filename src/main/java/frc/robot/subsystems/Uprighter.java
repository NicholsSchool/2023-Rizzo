package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.CANID;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

public class Uprighter extends SubsystemBase{

  private CANSparkMax uprighterMotorLeft;
  private CANSparkMax uprighterMotorRight;

  public Uprighter()
  {
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
  public void periodic()
  {

  }

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
