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

  private CANSparkMax sparkmax;
  private RelativeEncoder relEncoder;
  private SparkMaxPIDController pidController;
  private double setPoint;
  private TrapezoidProfile trapProfile;
  private Timer theTimer;
  private TrapezoidProfile.State targetState;
  private double feedForward;
  private double manualValue;

  public Arm() {

    setPoint = HOME_POSITION;
    sparkmax = new CANSparkMax(CANID.ARM_SPARKMAX, MotorType.kBrushless);
    sparkmax.setInverted(false);
    sparkmax.setSmartCurrentLimit(ARM_CURRENT_LIMIT);
    sparkmax.enableSoftLimit(SoftLimitDirection.kForward, true);
    sparkmax.enableSoftLimit(SoftLimitDirection.kReverse, true);
    sparkmax.setSoftLimit(SoftLimitDirection.kForward, (float) ARM_SOFT_LIMIT_FORWARD);
    sparkmax.setSoftLimit(SoftLimitDirection.kReverse, (float) ARM_SOFT_LIMIT_REVERSE);

    relEncoder = sparkmax.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    relEncoder.setPositionConversionFactor(ARM_POSITION_FACTOR);
    relEncoder.setVelocityConversionFactor(ARM_VELOCITY_FACTOR);

    pidController = sparkmax.getPIDController();
    RevPIDGains.setSparkMaxGains(pidController, ARM_POSITION_GAINS);
    sparkmax.burnFlash();

    theTimer = new Timer();
    theTimer.start();
    theTimer.reset();

    updateMotionProfile();
  }

  public void setTargetPosition(double _setpoint) {
    if (_setpoint != setPoint) {
      setPoint = _setpoint;
      updateMotionProfile();
    }
  }

  private void updateMotionProfile() {
    TrapezoidProfile.State state = new TrapezoidProfile.State(relEncoder.getPosition(), relEncoder.getVelocity());
    TrapezoidProfile.State goal = new TrapezoidProfile.State(setPoint, 0.0);
    trapProfile = new TrapezoidProfile(PROFILE_CONSTRAINTS, goal, state);
    theTimer.reset();
  }

  public void runAutomatic() {
    double elapsedTime = theTimer.get();
    if (trapProfile.isFinished(elapsedTime)) {
      targetState = new TrapezoidProfile.State(setPoint, 0.0);
    } else {
      targetState = trapProfile.calculate(elapsedTime);
    }
    feedForward = ARM_FEEDFORWARD.calculate(relEncoder.getPosition() + ARM_ZERO_COSINE_OFFSET, targetState.velocity);
    pidController.setReference(targetState.position, CANSparkMax.ControlType.kPosition, 0, feedForward);
  }

  public void runManual(double _power) {
    setPoint = relEncoder.getPosition();
    targetState = new TrapezoidProfile.State(setPoint, 0.0);
    trapProfile = new TrapezoidProfile(PROFILE_CONSTRAINTS, targetState, targetState);
    feedForward = ARM_FEEDFORWARD.calculate(relEncoder.getPosition() + ARM_ZERO_COSINE_OFFSET, targetState.velocity);
    sparkmax.set(_power + (feedForward / 12.0));
    manualValue = _power;
  }

  @Override
  public void periodic() {
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Final Setpoint", () -> setPoint, null);
    builder.addDoubleProperty("Position", () -> relEncoder.getPosition(), null);
    builder.addDoubleProperty("Applied Output", () -> sparkmax.getAppliedOutput(), null);
    builder.addDoubleProperty("Elapsed Time", () -> theTimer.get(), null);
    builder.addDoubleProperty("Target Position", () -> targetState.position, null);
    builder.addDoubleProperty("Manual Value", () -> manualValue, null);
  }
}
