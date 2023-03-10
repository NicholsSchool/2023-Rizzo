package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.UprighterConstants;

public class Uprighter extends SubsystemBase {

  private CANSparkMax uprighter;

  public Uprighter() {

    uprighter = new CANSparkMax(UprighterConstants.UPRIGHTER_ID, MotorType.kBrushless);
    uprighter.restoreFactoryDefaults();
    uprighter.setIdleMode(IdleMode.kBrake);
    uprighter.setInverted(false); // check if inversion needed

  }

  public void clockwise(double speed) {
    uprighter.set(speed);
  }

  public void counterClockwise(double speed) {
    uprighter.set(-speed);
  }

  public void stop() {
    uprighter.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
