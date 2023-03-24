package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.CBArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Uprighter;

public class AutoTest extends SequentialCommandGroup {
  /** Creates a new AutoTest. */
  public AutoTest( SwerveDrive swerveDrive, CBArm arm, Intake intake, Uprighter uprighter, Gripper gripper ) {

    double maxMPS = SwerveDriveConstants.MAX_METERS_PER_SECOND;
    double maxAMPS = AutoConstants.kMaxAccelerationMetersPerSecondSquared;
    double pTheta = AutoConstants.kPThetaController;
    double pX = AutoConstants.kPXController;
    double pY = AutoConstants.kPYController;

    TrajectoryConfig config = new TrajectoryConfig( maxMPS, maxAMPS).setKinematics( SwerveDriveConstants.SWERVE_DRIVE_KINEMATICS );

    Trajectory firstTrajectory = TrajectoryGenerator.generateTrajectory(
        // Zero the starting pose of the trajectory.
        new Pose2d(0, 0, new Rotation2d( 0 )),
        List.of(

            new Translation2d( 1, 0.1)
            //new Translation2d( -0.3, 0),
            //new Translation2d(-0.5, 0.01)
            //new Translation2d(-0.8, 0)
        // Add interior waypoints to the list above.
        ),
        // Final X/Y position in meters and rotation in radians.
        new Pose2d( 1.5, 0, new Rotation2d( 0 )),
        config);

    Trajectory secondTrajectory = TrajectoryGenerator.generateTrajectory(
      // Zero the starting pose of the trajectory.
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
          new Translation2d(0.5, 0)
          //new Translation2d(1, 0.5),
          //new Translation2d( 1.5, Math.PI / 2)
      // Add interior waypoints to the list above.
      ),
      // Final X/Y position in meters and rotation in radians.
      new Pose2d( 1, 0, new Rotation2d( 0 )),
      config);

      Trajectory thirdTrajectory = TrajectoryGenerator.generateTrajectory(
      // Zero the starting pose of the trajectory.
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
          new Translation2d(1, 0)
      // Add interior waypoints to the list above.
      ),
      // Final X/Y position in meters and rotation in radians.
      new Pose2d( 1.5, 0.1, new Rotation2d( 0 )),
      config);

    var thetaController = new ProfiledPIDController(pTheta, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  
    System.out.println( "Angle: " + swerveDrive.getHeading() );

    addRequirements(swerveDrive, arm, intake, gripper );
    addCommands(

      //Arm do stuff
      //new SwerveControllerCommand( firstTrajectory, swerveDrive::getPose, SwerveDriveConstants.SWERVE_DRIVE_KINEMATICS, new PIDController(pX, 0, 0),
      //new PIDController(pY, 0, 0), thetaController, swerveDrive::setModuleStates, swerveDrive).andThen( 
      new SwerveControllerCommand( firstTrajectory, swerveDrive::getPose, SwerveDriveConstants.SWERVE_DRIVE_KINEMATICS, new PIDController(pX, 0, 0),
      new PIDController(pY, 0, 0), thetaController, swerveDrive::setModuleStates, swerveDrive),
      new SwerveControllerCommand( secondTrajectory, swerveDrive::getPose, SwerveDriveConstants.SWERVE_DRIVE_KINEMATICS, new PIDController(pX, 0, 0),
      new PIDController(pY, 0, 0), thetaController, swerveDrive::setModuleStates, swerveDrive).raceWith(new IntakeDeploy(intake, uprighter, gripper).withTimeout( 1.5 ) )
      // new SwerveControllerCommand( secondTrajectory, swerveDrive::getPose, SwerveDriveConstants.SWERVE_DRIVE_KINEMATICS, new PIDController(pX, 0, 0),
      // new PIDController(pY, 0, 0), thetaController, swerveDrive::setModuleStates, swerveDrive),
      // new SwerveControllerCommand( thirdTrajectory, swerveDrive::getPose, SwerveDriveConstants.SWERVE_DRIVE_KINEMATICS, new PIDController(pX, 0, 0),
      // new PIDController(pY, 0, 0), thetaController, swerveDrive::setModuleStates, swerveDrive)
      
      );
  }
}
