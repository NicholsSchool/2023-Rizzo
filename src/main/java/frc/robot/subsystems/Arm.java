package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants.CANID;
import static frc.robot.Constants.ArmConstants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.*;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.AbsoluteEncoder;

public class Arm extends SubsystemBase {

  private double setpoint;
  private Timer timer;

  // absolute encoder attached to left uprighter SparkMax
  private AbsoluteEncoder armEncoder;

  private WPI_TalonFX falcon;

  public Arm() {
    falcon = new WPI_TalonFX(CANID.ARM_FALCON_FX);
    falcon.configFactoryDefault();
    falcon.setInverted(false);
    falcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    falcon.setNeutralMode(NeutralMode.Brake);
    falcon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
        true,
        STATOR_CURRENT_LIMIT,
        STATOR_TRIGGER_THRESHOLD_CURRENT,
        STATOR_TRIGGER_THRESHOLD_TIME));
    falcon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
        true,
        SUPPLY_CURRENT_LIMIT,
        SUPPLY_TRIGGER_THRESHOLD_CURRENT,
        STATOR_TRIGGER_THRESHOLD_TIME));

    // armEncoder = new AbsoluteEncoder(CANID.ARM_FALCON_FX);

    // turningEncoder = turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    // turningPIDController = turningSparkMax.getPIDController();
    // turningPIDController.setFeedbackDevice(turningEncoder);

    timer = new Timer();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setTargetPosition(double _setpoint) {
    if (_setpoint != setpoint) {
      setpoint = _setpoint;
      // updateMotionProfile();
    }
  }

}
