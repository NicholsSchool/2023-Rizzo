package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
// import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule {

  private final CANSparkMax m_drivingSparkMax;
  private final CANSparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkMaxPIDController m_drivingPIDController;
  private final SparkMaxPIDController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_drivingSparkMax.restoreFactoryDefaults();
    m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    m_drivingPIDController = m_drivingSparkMax.getPIDController();
    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // Calculations required for driving motor conversion factors and feed forward.
    double DRIVING_MOTOR_FREE_SPIN_RPM = 5676;
    double DRIVING_MOTOR_FREE_SPIN_RPS = DRIVING_MOTOR_FREE_SPIN_RPM / 60;
    double WHEEL_DIAMETER_IN_METERS = 0.0762;
    double WHEEL_CIRCUMFERENCE_IN_METERS = WHEEL_DIAMETER_IN_METERS * Math.PI;
    // 45 teeth on the driving wheel's bevel gear.
    // 22 teeth on the first-stage spur gear.
    // 15 teeth on the bevel pinion.
    double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (SwerveModuleConstants.DRIVING_MOTOR_PINION_TEETH * 15);
    double DRIVE_WHEEL_FREE_SPIN_RPS = (DRIVING_MOTOR_FREE_SPIN_RPS * WHEEL_CIRCUMFERENCE_IN_METERS)
        / DRIVING_MOTOR_REDUCTION;
    double DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER_IN_METERS * Math.PI)
        / DRIVING_MOTOR_REDUCTION;
    double DRIVING_ENCODER_VELOCITY_FACTOR = ((WHEEL_DIAMETER_IN_METERS * Math.PI)
        / DRIVING_MOTOR_REDUCTION) / 60.0;

    double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI);
    double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0;

    double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0;
    double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR;

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    m_drivingEncoder.setPositionConversionFactor(DRIVING_ENCODER_POSITION_FACTOR);
    m_drivingEncoder.setVelocityConversionFactor(DRIVING_ENCODER_VELOCITY_FACTOR);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_turningEncoder.setPositionConversionFactor(TURNING_ENCODER_POSITION_FACTOR);
    m_turningEncoder.setVelocityConversionFactor(TURNING_ENCODER_VELOCITY_FACTOR);

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    m_turningEncoder.setInverted(true);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(TURNING_ENCODER_POSITION_PID_MIN_INPUT);
    m_turningPIDController.setPositionPIDWrappingMaxInput(TURNING_ENCODER_POSITION_PID_MAX_INPUT);

    // Set the PID gains for the driving motor. Note these are example gains, and
    // you
    // may need to tune them for your own robot!
    m_drivingPIDController.setP(SwerveModuleConstants.DRIVING_P);
    m_drivingPIDController.setI(SwerveModuleConstants.DRIVING_I);
    m_drivingPIDController.setD(SwerveModuleConstants.DRIVING_D);
    m_drivingPIDController.setFF(SwerveModuleConstants.DRIVING_FF / DRIVE_WHEEL_FREE_SPIN_RPS);
    m_drivingPIDController.setOutputRange(SwerveModuleConstants.DRIVING_MIN_OUTPUT,
        SwerveModuleConstants.DRIVING_MAX_OUTPUT);

    // Set the PID gains for the turning motor. Note these are example gains, and
    // you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(SwerveModuleConstants.TURNING_P);
    m_turningPIDController.setI(SwerveModuleConstants.TURNING_I);
    m_turningPIDController.setD(SwerveModuleConstants.TURNING_D);
    m_turningPIDController.setFF(SwerveModuleConstants.TURNING_FF);
    m_turningPIDController.setOutputRange(SwerveModuleConstants.TURNING_MIN_OUTPUT,
        SwerveModuleConstants.TURNING_MAX_OUTPUT);

    m_drivingSparkMax.setIdleMode(SwerveModuleConstants.DRIVING_MOTOR_IDLE_MODE);
    m_turningSparkMax.setIdleMode(SwerveModuleConstants.TURNING_MOTOR_IDLE_MODE);
    m_drivingSparkMax.setSmartCurrentLimit(SwerveModuleConstants.DRIVING_MOTOR_CURRENT_LIMIT);
    m_turningSparkMax.setSmartCurrentLimit(SwerveModuleConstants.TURNING_MOTOR_CURRENT_LIMIT);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_drivingSparkMax.burnFlash();
    m_turningSparkMax.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

}
