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
import edu.wpi.first.wpilibj.Timer;
import static frc.robot.Constants.CANID;
import static frc.robot.Constants.ArmConstants.*;

public class Arm extends SubsystemBase {

  private CANSparkMax sparkMaxLeader;
  private CANSparkMax sparkMaxFollower;
  private RelativeEncoder relEncoder;
  private SparkMaxPIDController pidController;
  private double setPoint;
  private TrapezoidProfile trapProfile;
  private Timer theTimer;
  private TrapezoidProfile.State targetState;
  private double feedForward;

  public Arm() {

    setPoint = HOME_POSITION;
    sparkMaxLeader = new CANSparkMax(CANID.ARM_SPARKMAX_LEADER, MotorType.kBrushless);
    sparkMaxLeader.setInverted(false);

    sparkMaxFollower = new CANSparkMax(CANID.ARM_SPARKMAX_FOLLOWER, MotorType.kBrushless);
    sparkMaxFollower.setInverted(false);

    sparkMaxLeader.setSmartCurrentLimit(ARM_CURRENT_LIMIT);
    sparkMaxLeader.enableSoftLimit(SoftLimitDirection.kForward, true);
    sparkMaxLeader.enableSoftLimit(SoftLimitDirection.kReverse, true);
    sparkMaxLeader.setSoftLimit(SoftLimitDirection.kForward, (float) ARM_SOFT_LIMIT_FORWARD);
    sparkMaxLeader.setSoftLimit(SoftLimitDirection.kReverse, (float) ARM_SOFT_LIMIT_REVERSE);

    relEncoder = sparkMaxLeader.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    relEncoder.setPositionConversionFactor(ARM_POSITION_FACTOR);
    relEncoder.setVelocityConversionFactor(ARM_VELOCITY_FACTOR);

    pidController = sparkMaxLeader.getPIDController();
    RevPIDGains.setSparkMaxGains(pidController, ARM_POSITION_GAINS);
    sparkMaxFollower.follow(sparkMaxLeader);

    sparkMaxLeader.burnFlash();

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

  public void runAutomatic(double _power) {
    double deadband = 0.07;
    if (_power > deadband || _power < -deadband) {
      runManual(_power);
    } else {
      double elapsedTime = theTimer.get();
      if (trapProfile.isFinished(elapsedTime)) {
        targetState = new TrapezoidProfile.State(setPoint, 0.0);
      } else {
        targetState = trapProfile.calculate(elapsedTime);
      }
      feedForward = ARM_FEEDFORWARD.calculate(relEncoder.getPosition() + ARM_ZERO_COSINE_OFFSET, targetState.velocity);
      pidController.setReference(targetState.position, CANSparkMax.ControlType.kPosition, 0, feedForward);
    }
  }

  public void runManual(double _power) {
    setPoint = relEncoder.getPosition();
    targetState = new TrapezoidProfile.State(setPoint, 0.0);
    trapProfile = new TrapezoidProfile(PROFILE_CONSTRAINTS, targetState, targetState);
    feedForward = ARM_FEEDFORWARD.calculate(relEncoder.getPosition() + ARM_ZERO_COSINE_OFFSET, targetState.velocity);
    sparkMaxLeader.set(_power + (feedForward / 12.0));
  }

  public double getEncoderVal() {
    return relEncoder.getPosition();
  }

  @Override
  public void periodic() {
  }
}
