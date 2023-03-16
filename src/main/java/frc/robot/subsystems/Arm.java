package frc.robot.subsystems;

import frc.robot.utils.RevPIDGains;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import static frc.robot.Constants.CANID;
import static frc.robot.Constants.ArmConstants.*;

public class Arm extends SubsystemBase {

  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;
  private SparkMaxPIDController m_controller;
  private double m_setpoint;
  private TrapezoidProfile m_profile;
  private Timer m_timer;
  private TrapezoidProfile.State targetState;
  private double feedforward;
  private double manualValue;

  public Arm() {

    m_motor = new CANSparkMax(CANID.ARM_SPARKMAX, MotorType.kBrushless);
    m_motor.setInverted(false);
    m_motor.setSmartCurrentLimit(ARM_CURRENT_LIMIT);
    m_motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_motor.setSoftLimit(SoftLimitDirection.kForward, (float) ARM_SOFT_LIMIT_FORWARD);
    m_motor.setSoftLimit(SoftLimitDirection.kReverse, (float) ARM_SOFT_LIMIT_REVERSE);

    m_encoder = m_motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    m_encoder.setPositionConversionFactor(ARM_POSITION_FACTOR);
    m_encoder.setVelocityConversionFactor(ARM_VELOCITY_FACTOR);

    m_controller = m_motor.getPIDController();
    RevPIDGains.setSparkMaxGains(m_controller, ARM_POSITION_GAINS);

    m_motor.burnFlash();

    m_setpoint = HOME_POSITION;

    m_timer = new Timer();
    m_timer.start();
    m_timer.reset();

    updateMotionProfile();
  }

  public void setTargetPosition(double _setpoint) {
    if (_setpoint != m_setpoint) {
      m_setpoint = _setpoint;
      updateMotionProfile();
    }
  }

  private void updateMotionProfile() {
    TrapezoidProfile.State state = new TrapezoidProfile.State(m_encoder.getPosition(), m_encoder.getVelocity());
    TrapezoidProfile.State goal = new TrapezoidProfile.State(m_setpoint, 0.0);
    m_profile = new TrapezoidProfile(Constants.Arm.kArmMotionConstraint, goal, state);
    m_timer.reset();
  }

  public void runAutomatic() {
    double elapsedTime = m_timer.get();
    if (m_profile.isFinished(elapsedTime)) {
      targetState = new TrapezoidProfile.State(m_setpoint, 0.0);
    } else {
      targetState = m_profile.calculate(elapsedTime);
    }

    feedforward = Constants.Arm.kArmFeedforward.calculate(m_encoder.getPosition() + Constants.Arm.kArmZeroCosineOffset,
        targetState.velocity);
    m_controller.setReference(targetState.position, CANSparkMax.ControlType.kPosition, 0, feedforward);
  }

  public void runManual(double _power) {
    // reset and zero out a bunch of automatic mode stuff so exiting manual mode
    // happens cleanly and passively
    m_setpoint = m_encoder.getPosition();
    targetState = new TrapezoidProfile.State(m_setpoint, 0.0);
    m_profile = new TrapezoidProfile(Constants.Arm.kArmMotionConstraint, targetState, targetState);
    // update the feedforward variable with the newly zero target velocity
    feedforward = Constants.Arm.kArmFeedforward.calculate(m_encoder.getPosition() + Constants.Arm.kArmZeroCosineOffset,
        targetState.velocity);
    m_motor.set(_power + (feedforward / 12.0));
    manualValue = _power;
  }

  @Override
  public void periodic() { // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() { // This method will be called once per scheduler run during simulation

  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Final Setpoint", () -> m_setpoint, null);
    builder.addDoubleProperty("Position", () -> m_encoder.getPosition(), null);
    builder.addDoubleProperty("Applied Output", () -> m_motor.getAppliedOutput(), null);
    builder.addDoubleProperty("Elapsed Time", () -> m_timer.get(), null);
    /*
     * builder.addDoubleProperty("Target Position", () -> targetState.position,
     * null);
     * builder.addDoubleProperty("Target Velocity", () -> targetState.velocity,
     * null);
     */
    builder.addDoubleProperty("Feedforward", () -> feedforward, null);
    builder.addDoubleProperty("Manual Value", () -> manualValue, null);
    // builder.addDoubleProperty("Setpoint", () -> m_setpoint, (val) -> m_setpoint =
    // val);
    // builder.addBooleanProperty("At Setpoint", () -> atSetpoint(), null);
    // addChild("Controller", m_controller);
  }
}
