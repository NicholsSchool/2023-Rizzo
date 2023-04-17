package frc.robot.autos;

import java.util.HashMap;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.SwerveDriveConstants.*;

public class ElectricOneCube extends SequentialCommandGroup {

  SwerveDrive swerveDrive;
  Intake intake;
  Uprighter uprighter;
  Gripper gripper;
  Arm arm;

  public ElectricOneCube(SwerveDrive _swerveDrive, Intake _intake, Uprighter _uprighter, Gripper _gripper, Arm _arm) {

    swerveDrive = _swerveDrive;
    intake = _intake;
    uprighter = _uprighter;
    gripper = _gripper;
    arm = _arm;

    addRequirements(swerveDrive, intake, gripper, arm, uprighter);

    PathPlannerTrajectory path = PathPlanner.loadPath("ElectricForward", new PathConstraints(2, 3));

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(swerveDrive::getPose, swerveDrive::resetOdometry,
        SWERVE_DRIVE_KINEMATICS, new PIDConstants(3.0, 0.0, 0.0), new PIDConstants(0.5, 0.0, 0.0),
        swerveDrive::setModuleStates, new HashMap<String, Command>(), true, swerveDrive);

    addCommands(new RunCommand(() -> uprighter.spinOut(), intake).withTimeout(0.5),
        new OuttakeCube(intake, uprighter, gripper, OUTTAKE_HIGH_SPEED).withTimeout(0.5),
        autoBuilder.resetPose(path),
        autoBuilder.followPath(path),
        new RotateRobot(swerveDrive, 180.0).withTimeout(2),
        new MLCubePickup(swerveDrive).withTimeout(2.5).raceWith(new DeployIntake(intake, uprighter)));

  }

}
