package frc.robot.autos;

import java.util.HashMap;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import static frc.robot.Constants.SwerveDriveConstants.*;
import static frc.robot.Constants.IntakeConstants.*;

public class MayhemTwoCube extends SequentialCommandGroup {

  SwerveDrive swerveDrive;
  Intake intake;
  Uprighter uprighter;
  Gripper gripper;
  Arm arm;

  public MayhemTwoCube(SwerveDrive _swerveDrive, Intake _intake, Uprighter _uprighter, Gripper _gripper, Arm _arm) {

    swerveDrive = _swerveDrive;
    intake = _intake;
    uprighter = _uprighter;
    gripper = _gripper;
    arm = _arm;

    addRequirements(swerveDrive, intake, gripper, arm, uprighter);

    PathPlannerTrajectory forward = PathPlanner.loadPath("MayhemForward", new PathConstraints(4, 3));
    PathPlannerTrajectory backward = PathPlanner.loadPath("MayhemBackward", new PathConstraints(4, 3));

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(swerveDrive::getPose, swerveDrive::resetOdometry,
        SWERVE_DRIVE_KINEMATICS, new PIDConstants(3.0, 0.0, 0.0), new PIDConstants(0.5, 0.0, 0.0),
        swerveDrive::setModuleStates, new HashMap<String, Command>(), true, swerveDrive);

    addCommands(new RunCommand(() -> uprighter.spinOut(), intake).withTimeout(0.5),
        new OuttakeCube(intake, uprighter, gripper, OUTTAKE_HIGH_SPEED).withTimeout(0.5),
        autoBuilder.resetPose(forward),
        autoBuilder.followPath(forward),
        new RotateRobot(swerveDrive, 180.0).withTimeout(2),
        new MLCubePickup(swerveDrive).withTimeout(0.75).raceWith(new DeployIntake(intake, uprighter)),
        new RotateRobot(swerveDrive, 0.0).withTimeout(2),
        autoBuilder.resetPose(backward),
        autoBuilder.followPath(backward),
        new ApriltagAlign(swerveDrive).withTimeout(2),
        new OuttakeCube(intake, uprighter, gripper, OUTTAKE_LOW_SPEED).withTimeout(2),
        new InstantCommand(() -> swerveDrive.setGyroAngleAdjustment(180.0)));

  }

}
