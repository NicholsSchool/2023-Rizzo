package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.RevPIDGains;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.lang.Math;

public final class Constants {

  public static final int MILLISECS_PER_SEC = 1000;

  // Intake/Lifter/Uprighter
  public static final class IntakeConstants {
    public static final int INTAKE_PISTON_SOLENOID_CHANNEL = 1;
    public static final boolean INTAKE_CLOSED = true;
    public static final double INTAKE_SPEED = 0.8;
    public static final int LIFTER_PISTON_SOLENOID_CHANNEL = 2;
    public static final boolean LIFTER_UP = true;
    public static final double UPRIGHTER_SPEED = 1.0;
  }

  // Arm (Manipulator)
  public static final class ArmConstants {

    public static final double HOME_POSITION = 0.0;
    public static final double SCORING_POSITION = 3.05;
    public static final double INTAKE_POSITION = 4.52;
    public static final double FEEDER_POSITION = 2.95;

    public static final int ARM_CURRENT_LIMIT = 40;
    public static final double ARM_SOFT_LIMIT_FORWARD = 4.6;
    public static final double ARM_SOFT_LIMIT_REVERSE = 0.0;

    public static final double ARM_GEAR_RATIO = 1.0 / (80 * 1.6); // gear box ratio * belt offset
    public static final double ARM_POSITION_FACTOR = ARM_GEAR_RATIO * 2.0 * Math.PI;
    public static final double ARM_VELOCITY_FACTOR = ARM_GEAR_RATIO * 2.0 * Math.PI / 60.0;
    public static final double ARM_FREE_SPEED = 5676.0 * ARM_VELOCITY_FACTOR;

    public static final double ARM_ZERO_COSINE_OFFSET = -Math.PI / 6; // radians offset from zeroing arm
    public static final ArmFeedforward ARM_FEEDFORWARD = new ArmFeedforward(0.0, 0.4, (12.0 / ARM_FREE_SPEED), 0.0);
    public static final RevPIDGains ARM_POSITION_GAINS = new RevPIDGains(0.6, 0.0, 0.0);
    public static final TrapezoidProfile.Constraints PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(2.0, 2.0);

  }

  // Gripper/Pinchers/Spinners (End Effector)
  public static final class GripperConstants {
    public static final int PINCHER_SOLENOID_CHANNEL = 3;
    public static final boolean PINCHER_CLOSED = true;
    public static final double GRIPPER_SPEED = 0.4;
  }

  // Controller Area Network (CAN) IDs
  public static final class CANID {
    public static final int REAR_RIGHT_DRIVING_SPARKMAX = 10;
    public static final int REAR_RIGHT_TURNING_SPARKMAX = 11;
    public static final int FRONT_RIGHT_DRIVING_SPARKMAX = 12;
    public static final int FRONT_RIGHT_TURNING_SPARKMAX = 13;
    public static final int FRONT_LEFT_DRIVING_SPARKMAX = 14;
    public static final int FRONT_LEFT_TURNING_SPARKMAX = 15;
    public static final int REAR_LEFT_DRIVING_SPARKMAX = 16;
    public static final int REAR_LEFT_TURNING_SPARKMAX = 17;
    public static final int LEFT_INTAKE_SPARKMAX = 20;
    public static final int RIGHT_INTAKE_SPARKMAX = 21;
    public static final int LEFT_UPRIGHTER_SPARKMAX = 22;
    public static final int RIGHT_UPRIGHTER_SPARKMAX = 23;
    public static final int GRIPPER_SPARKMAX = 24;
    public static final int ARM_SPARKMAX = 25;
  }

  // Swerve Drive (Drive Train)
  public static final class SwerveDriveConstants {
    public static final double DRIVETRAIN_WIDTH = Units.inchesToMeters(26.5);
    public static final double DRIVETRAIN_LENGTH = Units.inchesToMeters(26.5);

    public static final double VIRTUAL_LOW_GEAR_RATE = 0.6;
    public static final double VIRTUAL_HIGH_GEAR_RATE = 1.0;

    public static final double MAX_METERS_PER_SECOND = 4.8;
    public static final double MAX_ANGULAR_SPEED = 2 * Math.PI;

    public static final double ROTATIONAL_SLEW_RATE = 2.0; // percent per second (1 = 100%)

    public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(DRIVETRAIN_LENGTH / 2, DRIVETRAIN_WIDTH / 2),
        new Translation2d(DRIVETRAIN_LENGTH / 2, -DRIVETRAIN_WIDTH / 2),
        new Translation2d(-DRIVETRAIN_LENGTH / 2, DRIVETRAIN_WIDTH / 2),
        new Translation2d(-DRIVETRAIN_LENGTH / 2, -DRIVETRAIN_WIDTH / 2));
  }

  // REV MAXSwerve Modules
  public static final class SwerveModuleConstants {
    public static final int DRIVING_MOTOR_PINION_TEETH = 12; // 12T, 13T, or 14T
    public static final double DRIVING_MOTOR_FREE_SPIN_RPM = 5676; // NEO 550s max RPM
    public static final double WHEEL_DIAMETER_IN_METERS = 0.0762; // 3 inch wheels

    public static final double DRIVING_P = 0.04;
    public static final double DRIVING_I = 0;
    public static final double DRIVING_D = 0;
    public static final double DRIVING_FF = 1; // later offset by free spin rate

    public static final double TURNING_P = 1;
    public static final double TURNING_I = 0;
    public static final double TURNING_D = 0;
    public static final double TURNING_FF = 0;

    public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

    public static final int DRIVING_MOTOR_CURRENT_LIMIT = 17; // amps
    public static final int TURNING_MOTOR_CURRENT_LIMIT = 12; // amps
  }

  public static final class WebcamConstants {
    public static final double W = 160.0; // Pixels
    public static final double H = 120.0;
    public static final double D_FOV = 1.20; // Radians
    public static final double H_FOV = 1.05;
    public static final double V_FOV = 0.59;
    public static final double H_RADIANS_PER_PIXEL = H_FOV / W; // Radians / Pixel
    public static final double V_RADIANS_PER_PIXEL = V_FOV / H;
  }

  public static final class ConeConstants {
    public static final double GREATER_DIM = 0.33; // Meters
    public static final double LESSER_DIM = 0.21;
  }

  public static final class CubeConstants {
    public static final double SIZE = 0.24; // Meters
  }
}
