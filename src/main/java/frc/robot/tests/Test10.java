package frc.robot.tests;

import java.util.HashMap;
import java.util.List;
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
// import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import com.pathplanner.lib.PathConstraints;

public class Test10 extends SequentialCommandGroup {

  SwerveDrive swerveDrive;
  Intake intake;
  Uprighter uprighter;
  Gripper gripper;
  Arm arm;

  public Test10(SwerveDrive _swerveDrive, Intake _intake, Uprighter _uprighter, Gripper _gripper, Arm _arm) {

    swerveDrive = _swerveDrive;
    intake = _intake;
    uprighter = _uprighter;
    gripper = _gripper;
    arm = _arm;

    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Test10", new PathConstraints(3, 2));

    HashMap<String, Command> eventMap = new HashMap<>();

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(_swerveDrive::getPose, _swerveDrive::resetOdometry,
        SwerveDriveConstants.SWERVE_DRIVE_KINEMATICS, new PIDConstants(3.0, 0.01, 0.0), new PIDConstants(0.5, 0.0, 0.0),
        _swerveDrive::setModuleStates, eventMap, true, _swerveDrive);

    addRequirements(swerveDrive, intake, gripper, arm, uprighter);

    Command gotoChargeStation = autoBuilder.fullAuto(pathGroup);

    addCommands(gotoChargeStation,
        new RunCommand(() -> new BalanceRobot(swerveDrive, 0), swerveDrive).withTimeout(5.0));

  }
}
