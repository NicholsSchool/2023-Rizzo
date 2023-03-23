package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.CANID;
import static frc.robot.Constants.ArmConstants.*;

public class Arm extends SubsystemBase {

  private CANSparkMax armMotor;
  private RelativeEncoder armEncoder;
  private DigitalInput armLimitSwitch;
  private SparkMaxPIDController armPIDController;
  private double armSetpoint;
  private boolean armAtLimit;
  private TrapezoidProfile motorProfile;
  private TrapezoidProfile.State targetState;
  private double feedforward;
  private Timer timer;

  public Arm() {

    armMotor = new CANSparkMax(CANID.ARM_SPARKMAX, MotorType.kBrushless);
    armEncoder = armMotor.getEncoder(Type.kHallSensor, 42);
    armLimitSwitch = new DigitalInput(ARM_LIMIT_SWITCH_DIO_CHANNEL);

    armMotor.setInverted(false);
    armMotor.setSmartCurrentLimit(ARM_CURRENT_LIMIT);
    armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    armMotor.setSoftLimit(SoftLimitDirection.kForward, (float) SOFT_LIMIT_FORWARD);
    armMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) SOFT_LIMIT_REVERSE);
    armMotor.setIdleMode(IdleMode.kCoast); // while testing, set to kCoast
    // armMotor.setIdleMode(IdleMode.kBrake);

    armEncoder.setInverted(false);
    armEncoder.setPositionConversionFactor(POSITION_CONVERSION_FACTOR);
    armEncoder.setVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);

    armAtLimit = armLimitSwitch.get();
    resetEncoder();

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

  @Override
  public void periodic() {
    resetEncoderAtLimit();
    System.out.println("Arm Position: " + armEncoder.getPosition());
  }

  /**
   * Sets the arm to a target position.
   * 
   * @param setpoint
   */
  public void setTargetPosition(double setpoint) {
    if (setpoint != armSetpoint) {
      armSetpoint = setpoint;
      updateMotionProfile();
    }
  }

  /**
   * Updates the motion profile with the current arm position and velocity.
   */
  private void updateMotionProfile() {
    TrapezoidProfile.State state = new TrapezoidProfile.State(armEncoder.getPosition(), armEncoder.getVelocity());
    TrapezoidProfile.State goal = new TrapezoidProfile.State(armSetpoint, 0.0);
    motorProfile = new TrapezoidProfile(kArmMotionConstraint, goal, state);
    timer.reset();
  }

  /**
   * Automatically moves the arm to the target position.
   */
  public void runAutomatic() {
    double elapsedTime = timer.get();
    if (motorProfile.isFinished(elapsedTime)) {
      targetState = new TrapezoidProfile.State(armSetpoint, 0.0);
    } else {
      targetState = motorProfile.calculate(elapsedTime);
    }
    feedforward = ARM_FF.calculate(armEncoder.getPosition() + ARM_ZERO_COSINE_OFFSET, targetState.velocity);
    armPIDController.setReference(targetState.position, CANSparkMax.ControlType.kPosition, 0, feedforward);
  }

  /**
   * Manually moves the arm with a given power.
   * 
   * @param power
   */
  public void runManual(double power) {
    armSetpoint = armEncoder.getPosition();
    targetState = new TrapezoidProfile.State(armSetpoint, 0.0);
    motorProfile = new TrapezoidProfile(kArmMotionConstraint, targetState, targetState);
    // update the feedforward variable with the new target state
    feedforward = ARM_FF.calculate(armEncoder.getPosition() + ARM_ZERO_COSINE_OFFSET, targetState.velocity);
    // set arm motor speed to manual control with scaled power
    armMotor.set((power * ARM_MANUAL_SCALED) + (feedforward / 12.0));
  }

  /**
   * Checks if the limit switch is pressed.
   */
  public boolean isAtLimit() {
    if (armLimitSwitch.get() == armAtLimit) {
      return true;
    } else {
      return false;
    }
  }

  public void resetEncoder() {
    armEncoder.setPosition(0.0);
  }

  /**
   * Resets the encoder if the limit switch is pressed.
   */
  public void resetEncoderAtLimit() {
    if (isAtLimit()) {
      resetEncoder();
      System.out.println("Resetting Arm Encoder to 0.0");
    }
  }

}
