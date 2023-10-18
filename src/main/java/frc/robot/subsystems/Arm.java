package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CANID;
import static frc.robot.Constants.ArmConstants.*;

public class Arm extends SubsystemBase {

  private CANSparkMax armMotor;
  private RelativeEncoder armEncoder;

  public Arm() {

    // arm Motor
    armMotor = new CANSparkMax(CANID.ARM_SPARKMAX, MotorType.kBrushless);
    armMotor.restoreFactoryDefaults();
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setInverted(true);
    armMotor.enableSoftLimit(SoftLimitDirection.kForward, false); //TODO: Put limits back
    armMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    armMotor.setSoftLimit(SoftLimitDirection.kForward, (float) (MAX_ARM_LIMIT) );
    armMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) (MIN_ARM_LIMIT) );
    armEncoder = armMotor.getEncoder(Type.kHallSensor, ARM_COUNTS_PER_REV);
    armEncoder.setPosition(0.0);
    armEncoder.setPositionConversionFactor(ArmConstants.POSITION_CONVERSION_FACTOR);
    armMotor.burnFlash();
  }

  @Override
  public void periodic() {
    armValuesToNT();
  }

  /**
   * Put arm values on network tables.
   */
  public void armValuesToNT() {
    RobotContainer.armEncoder.setDouble(getArmRotations());
  }

  public double getArmRotations() {
    return armEncoder.getPosition();
  }

  public void resetEncoder() {
    armEncoder.setPosition(0.0);
  }

  public void spin( double speed ) {
    armMotor.set(-speed);
  }

  public void goToAngle(double desiredPos) {
    double armAngle = getArmRotations();
    armMotor.set( ARM_P * (desiredPos - armAngle) );
  }

  public void stop() {
    armMotor.stopMotor();
  }
}