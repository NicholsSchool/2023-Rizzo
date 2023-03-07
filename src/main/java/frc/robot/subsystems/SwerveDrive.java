package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {

  // Create all REV MAXSwerve Modules
  private final SwerveModule m_frontLeft = new SwerveModule(
      CANID.FRONT_LEFT_DRIVING_SPARKMAX,
      CANID.FRONT_LEFT_TURNING_SPARKMAX,
      (-Math.PI / 2));

  private final SwerveModule m_frontRight = new SwerveModule(
      CANID.FRONT_RIGHT_DRIVING_SPARKMAX,
      CANID.FRONT_RIGHT_DRIVING_SPARKMAX,
      (0));

  private final SwerveModule m_rearLeft = new SwerveModule(
      CANID.REAR_LEFT_DRIVING_SPARKMAX,
      CANID.REAR_LEFT_TURNING_SPARKMAX,
      (Math.PI));

  private final SwerveModule m_rearRight = new SwerveModule(
      CANID.REAR_RIGHT_DRIVING_SPARKMAX,
      CANID.REAR_RIGHT_TURNING_SPARKMAX,
      (Math.PI / 2));

  // Create the IMU sensor
  private final AHRS navX = new AHRS(SPI.Port.kMXP);

  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(SwerveDriveConstants.ROTATIONAL_SLEW_RATE);

  // Create odometry object for tracking robot pose.
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      SwerveDriveConstants.DRIVETRAIN_KINEMATICS,
      Rotation2d.fromDegrees(transfromAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public SwerveDrive() {
    navX.calibrate();
  }

  @Override
  public void periodic() {
    // Update the odometry with esitmated robot pose.
    m_odometry.update(
        Rotation2d.fromDegrees(transfromAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    // System.out.println(transfromAngle());
  }

  public double transfromAngle() {
    return navX.getAngle();
  }

  public void resetGyro() {
    navX.zeroYaw();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(transfromAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded = xSpeed;
    double ySpeedCommanded = ySpeed;
    double m_currentRotation;

    if (rateLimit) {
      m_currentRotation = m_rotLimiter.calculate(rot);
    } else {
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * SwerveDriveConstants.MAX_METERS_PER_SECOND;
    double ySpeedDelivered = ySpeedCommanded * SwerveDriveConstants.MAX_METERS_PER_SECOND;
    double rotDelivered = m_currentRotation * SwerveDriveConstants.MAX_ANGULAR_SPEED;

    var swerveModuleStates = SwerveDriveConstants.DRIVETRAIN_KINEMATICS.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(transfromAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, SwerveDriveConstants.MAX_METERS_PER_SECOND);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, SwerveDriveConstants.MAX_METERS_PER_SECOND);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    navX.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(transfromAngle()).getDegrees();
  }

}
