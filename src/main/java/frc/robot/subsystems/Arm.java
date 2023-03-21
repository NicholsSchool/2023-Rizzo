package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.CANID;
import static frc.robot.Constants.ArmConstants.*;

public class Arm extends SubsystemBase {

  private CANSparkMax armMotor;
  private RelativeEncoder armEncoder;
  private SparkMaxPIDController armPIDController;
  private double armSetpoint;
  private TrapezoidProfile motorProfile;
  private TrapezoidProfile.State targetState;
  private double feedforward;
  private Timer timer;

  public Arm() {

    armMotor = new CANSparkMax(CANID.ARM_SPARKMAX, MotorType.kBrushless);

    armMotor.setInverted(false);
    armMotor.setSmartCurrentLimit(kCurrentLimit);
    armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    armMotor.setSoftLimit(SoftLimitDirection.kForward, (float) kSoftLimitForward);
    armMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) kSoftLimitReverse);
    armMotor.setIdleMode(IdleMode.kCoast);
    // armMotor.setIdleMode(IdleMode.kBrake);

    armEncoder = armMotor.getEncoder(Type.kHallSensor, 42);
    armEncoder.setPositionConversionFactor(kPositionFactor);
    armEncoder.setVelocityConversionFactor(kVelocityFactor);

    armPIDController = armMotor.getPIDController();
    armPIDController.setP(ARM_DEFAULT_P);
    armPIDController.setI(ARM_DEFAULT_I);
    armPIDController.setD(ARM_DEFAULT_D);

    armMotor.burnFlash();

    armSetpoint = HOME_POSITION;

    timer = new Timer();
    timer.start();
    timer.reset();

    updateMotionProfile();
  }

  public void setTargetPosition(double _setpoint) {
    if (_setpoint != armSetpoint) {
      armSetpoint = _setpoint;
      updateMotionProfile();
    }
  }

  private void updateMotionProfile() {
    TrapezoidProfile.State state = new TrapezoidProfile.State(armEncoder.getPosition(), armEncoder.getVelocity());
    TrapezoidProfile.State goal = new TrapezoidProfile.State(armSetpoint, 0.0);
    motorProfile = new TrapezoidProfile(kArmMotionConstraint, goal, state);
    timer.reset();
  }

  public void runAutomatic() {
    double elapsedTime = timer.get();
    if (motorProfile.isFinished(elapsedTime)) {
      targetState = new TrapezoidProfile.State(armSetpoint, 0.0);
    } else {
      targetState = motorProfile.calculate(elapsedTime);
    }
    feedforward = kArmFeedforward.calculate(armEncoder.getPosition() + kArmZeroCosineOffset, targetState.velocity);
    armPIDController.setReference(targetState.position, CANSparkMax.ControlType.kPosition, 0, feedforward);
  }

  public void runManual(double power) {
    // reset and set to zero to exiting manual mode cleanly
    armSetpoint = armEncoder.getPosition();
    targetState = new TrapezoidProfile.State(armSetpoint, 0.0);
    motorProfile = new TrapezoidProfile(kArmMotionConstraint, targetState, targetState);
    // update the feedforward variable with the new target state
    feedforward = kArmFeedforward.calculate(armEncoder.getPosition() + kArmZeroCosineOffset, targetState.velocity);
    armMotor.set(power + (feedforward / 12.0));
  }

  @Override
  public void periodic() {
    System.out.println("Arm Position: " + armEncoder.getPosition());
  }
}
