package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.CANID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.UprighterConstants.*;

public class Uprighter extends SubsystemBase {

  private CANSparkMax uprighterMotorLeft;
  private CANSparkMax uprighterMotorRight;

  public Uprighter() {
    // Uprighter Motor Left
    uprighterMotorLeft = new CANSparkMax(CANID.LEFT_UPRIGHTER_SPARKMAX, MotorType.kBrushless);
    uprighterMotorLeft.restoreFactoryDefaults();
    uprighterMotorLeft.setIdleMode(IdleMode.kBrake);
    uprighterMotorLeft.setInverted(false);

    // Uprighter Motor Right
    uprighterMotorRight = new CANSparkMax(CANID.RIGHT_UPRIGHTER_SPARKMAX, MotorType.kBrushless);
    uprighterMotorRight.restoreFactoryDefaults();
    uprighterMotorRight.setIdleMode(IdleMode.kBrake);
    uprighterMotorRight.setInverted(false);
  }

  @Override
  public void periodic() {
  }

  public void spinIn() {
    uprighterMotorLeft.set(-UPRIGHTER_SPEED);
    uprighterMotorRight.set(UPRIGHTER_SPEED);
  }

  public void spinOut() {
    uprighterMotorLeft.set(UPRIGHTER_SPEED);
    uprighterMotorRight.set(-UPRIGHTER_SPEED);
  }

  public void stop() {
    uprighterMotorLeft.stopMotor();
    uprighterMotorRight.stopMotor();
  }

}
