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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANID;
import static frc.robot.Constants.SwerveDriveConstants.*;

public class SwerveDrive extends SubsystemBase {

  private final SwerveModule frontLeftModule = new SwerveModule(
      CANID.FRONT_LEFT_DRIVING_SPARKMAX,
      CANID.FRONT_LEFT_TURNING_SPARKMAX,
      (-Math.PI / 2));

  private final SwerveModule frontRightModule = new SwerveModule(
      CANID.FRONT_RIGHT_DRIVING_SPARKMAX,
      CANID.FRONT_RIGHT_TURNING_SPARKMAX,
      (0));

  private final SwerveModule rearLeftModule = new SwerveModule(
      CANID.REAR_LEFT_DRIVING_SPARKMAX,
      CANID.REAR_LEFT_TURNING_SPARKMAX,
      (Math.PI));

  private final SwerveModule rearRightModule = new SwerveModule(
      CANID.REAR_RIGHT_DRIVING_SPARKMAX,
      CANID.REAR_RIGHT_TURNING_SPARKMAX,
      (Math.PI / 2));

  // Attitude and Heading Reference System (AHRS)
  private final AHRS navX = new AHRS(SPI.Port.kMXP);

  // Setup rate limiters for translation and rotation.
  private double translationRateLimiter = VIRTUAL_LOW_GEAR_RATE;
  private SlewRateLimiter rotationRateLimiter = new SlewRateLimiter(ROTATIONAL_SLEW_RATE);

  // Create odometry object for tracking robot pose.
  SwerveDriveOdometry robotOdometry = new SwerveDriveOdometry(
      SWERVE_DRIVE_KINEMATICS,
      Rotation2d.fromDegrees(getContinuousAngle()),
      new SwerveModulePosition[] {
          frontLeftModule.getPosition(),
          frontRightModule.getPosition(),
          rearLeftModule.getPosition(),
          rearRightModule.getPosition()
      });

  /** Constructor for a new SwerveDrive Subsystem. */
  public SwerveDrive() {
    // Note: NavX calibration takes approximately 3 to 4 seconds.
    navX.calibrate();
  }

  @Override
  public void periodic() {
    // Update the odometry with esitmated robot pose.
    // System.out.println(getContinuousAngle());
    robotOdometry.update(
        Rotation2d.fromDegrees(getContinuousAngle()),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            rearLeftModule.getPosition(),
            rearRightModule.getPosition()
        });
  }

  /** Get the current continuously accruing angle of the AHRS. */
  public double getContinuousAngle() {
    return -navX.getAngle();
  }

  /** Reset the AHRS to zero. */
  public void resetGyro() {
    navX.zeroYaw();
  }

  /** Set the gear ratio to High Gear */
  public void setVirtualHighGear() {
    translationRateLimiter = VIRTUAL_HIGH_GEAR_RATE;
    System.out.println("setting virtual gear to high");
  }

  /** Set the gear ratio to Low Gear */
  public void setVirtualLowGear() {
    translationRateLimiter = VIRTUAL_LOW_GEAR_RATE;
    System.out.println("setting virtual gear to low");
  }

  /** Returns the currently-estimated pose of the robot. */
  public Pose2d getPose() {
    return robotOdometry.getPoseMeters();
  }

  /** Resets the odometry to the specified pose. */
  public void resetOdometry(Pose2d pose) {
    robotOdometry.resetPosition(
        Rotation2d.fromDegrees(getContinuousAngle()),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            rearLeftModule.getPosition(),
            rearRightModule.getPosition()
        }, pose);
  }

  /**
   * Drive the robot based on input from either a joystick or auto routine.
   *
   * @param xSpeed          Speed of the robot in the x direction (forward).
   * @param ySpeed          Speed of the robot in the y direction (sideways).
   * @param angularRotation Angular rate of robot chasis rotation.
   * @param fieldRelative   Whether x and y speeds are field relative, or not.
   */
  public void drive(double xSpeed, double ySpeed, double angularRotation, boolean fieldRelative) {

    // Note: This Rotational Rate Limiter is experimental.
    angularRotation = rotationRateLimiter.calculate(angularRotation);

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * MAX_METERS_PER_SECOND * translationRateLimiter;
    double ySpeedDelivered = ySpeed * MAX_METERS_PER_SECOND * translationRateLimiter;
    double rotDelivered = angularRotation * MAX_ANGULAR_SPEED;

    SwerveModuleState[] swerveModuleStates;

    if (fieldRelative) {
      // field relative driving
      swerveModuleStates = SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
              Rotation2d.fromDegrees(getContinuousAngle())));
    } else {
      // robot orientated driving
      swerveModuleStates = SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
          new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_METERS_PER_SECOND);

    frontLeftModule.setDesiredState(swerveModuleStates[0]);
    frontRightModule.setDesiredState(swerveModuleStates[1]);
    rearLeftModule.setDesiredState(swerveModuleStates[2]);
    rearRightModule.setDesiredState(swerveModuleStates[3]);
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setWheelsToXFormation() {
    frontLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /** Sets the swerve ModuleStates. */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_METERS_PER_SECOND);
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    rearLeftModule.setDesiredState(desiredStates[2]);
    rearRightModule.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of zero. */
  public void resetEncoders() {
    frontLeftModule.resetEncoders();
    rearLeftModule.resetEncoders();
    frontRightModule.resetEncoders();
    rearRightModule.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    navX.reset();
  }

  /**
   * Returns the heading of the robot.
   * 
   * @return The current yaw value in degrees (-180 to 180).
   */
  public double getYaw() {
    return navX.getYaw();
  }

  /** Returns the heading of the robot in degrees, from -180 to 180 */
  public double getHeading() {
    return Rotation2d.fromDegrees(getContinuousAngle()).getDegrees();
  }

}
