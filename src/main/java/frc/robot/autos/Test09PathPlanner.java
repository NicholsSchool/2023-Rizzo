package frc.robot.autos;

import java.util.HashMap;
import java.util.List;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import com.pathplanner.lib.PathConstraints;

public class Test09PathPlanner extends SequentialCommandGroup {


  PhotonCamera camera; 
  SwerveDrive swerveDrive;
  Intake intake;
  Uprighter uprighter;
  Gripper gripper;
  Arm arm;

  public Test09PathPlanner(SwerveDrive _swerveDrive, Intake _intake, Uprighter _uprighter, Gripper _gripper, Arm _arm) {

    swerveDrive = _swerveDrive;
    intake = _intake;
    uprighter = _uprighter;
    gripper = _gripper;
    arm = _arm;

    camera = new PhotonCamera( "Microsoft_LifeCam_HD-3000" );


    PathPlannerTrajectory path = PathPlanner.loadPath("Test10", new PathConstraints(4.0, 3));

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(_swerveDrive::getPose, _swerveDrive::resetOdometry,
        SwerveDriveConstants.SWERVE_DRIVE_KINEMATICS, new PIDConstants(3.0, 0.0, 0.0), new PIDConstants(0.5, 0.0, 0.0),
        _swerveDrive::setModuleStates, new HashMap<String, Command>(), true, _swerveDrive);

    addRequirements(swerveDrive, intake, gripper, arm, uprighter);

    // Command gotoChargeStation = autoBuilder.fullAuto(path);

    addCommands(new RunCommand(() -> intake.close(), intake).withTimeout(0.5),
        new RunCommand(() -> uprighter.spinOut(), intake).withTimeout(0.75),
        new InstantCommand(() -> uprighter.stop(), intake),
        new RunCommand(() -> intake.spinOut(), intake).withTimeout(0.5),
        new InstantCommand(() -> intake.open(), intake),
        new InstantCommand(() -> intake.stop(), intake));
    addCommands(autoBuilder.resetPose(path));
    addCommands(autoBuilder.followPath(path));
    addCommands(new BalanceRobot(swerveDrive, false).withTimeout(11.0));

  }
}
