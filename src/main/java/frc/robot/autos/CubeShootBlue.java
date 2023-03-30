// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import javax.swing.SpringLayout.Constraints;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.commands.OuttakeCube;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Uprighter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CubeShootBlue extends SequentialCommandGroup {

  /** Creates a new CubeShootBlue. */
  public CubeShootBlue(SwerveDrive swerveDrive, Intake intake, Uprighter uprighter, Gripper gripper, Arm arm) {

    // TrajectoryConfig config = new TrajectoryConfig(Math.PI, Math.PI)
    // .setKinematics(SwerveDriveConstants.SWERVE_DRIVE_KINEMATICS);

    // Trajectory backUpOneMeter = TrajectoryGenerator.generateTrajectory(
    // // Zero the starting pose of the trajectory.
    // new Pose2d(0, 0, new Rotation2d(0)),
    // List.of(
    // new Translation2d(0.5, 0),
    // // new Translation2d(1, 0.5),
    // // new Translation2d( 1.5, Math.PI / 2)
    // // Add interior waypoints to the list above.
    // new Translation2d( -0.5, 0.1 )
    // ),
    // // Final X/Y position in meters and rotation in radians.
    // new Pose2d(1, 0, new Rotation2d(0) ), config);

    // ProfiledPIDController thetaController = new ProfiledPIDController(1.0, 0, 0,
    // new Constraints(Math.PI, Math.PI));
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addCommands(
        new RunCommand(() -> intake.lower(), intake).withTimeout(1.0),
        new RunCommand(() -> intake.raise(), intake).withTimeout(2.0),
        new RunCommand(() -> intake.spinOut(), intake).withTimeout(5),
        new InstantCommand(() -> intake.stop(), intake)

    );
  }
}
