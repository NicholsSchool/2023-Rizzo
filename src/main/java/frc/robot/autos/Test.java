package frc.robot.autos;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.commands.DeployIntake;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import com.pathplanner.lib.PathConstraints;





public class Test extends SequentialCommandGroup {
  
  SwerveDrive swerveDrive;
  Intake intake;
  Uprighter uprighter;
  Gripper gripper;
  Arm arm;

  public Test(SwerveDrive _swerveDrive, Intake _intake, Uprighter _uprighter, Gripper _gripper, Arm _arm) {
    
    swerveDrive = _swerveDrive;
    intake = _intake;
    uprighter = _uprighter;
    gripper = _gripper;
    arm = _arm;



    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("test", new PathConstraints(4, 3));

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("intake 1", new DeployIntake(_intake, _uprighter). withTimeout( 3 ));
    eventMap.put("intake 2", new DeployIntake(_intake, _uprighter). withTimeout( 3 ));


    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(_swerveDrive::getPose, _swerveDrive::resetOdometry,
        SwerveDriveConstants.SWERVE_DRIVE_KINEMATICS, new PIDConstants(3.0, 0.0, 0.0), new PIDConstants(0.5, 0.0, 0.0),
        _swerveDrive::setModuleStates, eventMap, true, _swerveDrive);

    addRequirements(swerveDrive, intake, gripper, arm, uprighter);

    Command fullAuto = autoBuilder.fullAuto(pathGroup);

    addCommands( 
      autoBuilder.resetPose(pathGroup.get(0)),
        new RunCommand(() -> intake.close(), intake).withTimeout(0.5),
        new RunCommand(() -> uprighter.spinOut(), intake).withTimeout(0.65),
        new RunCommand(() -> uprighter.stop(), intake).withTimeout(0.0),
        new RunCommand(() -> intake.spinOut(), intake).withTimeout(0.5),
        new RunCommand(() -> intake.open(), intake).withTimeout(0.5),
        new InstantCommand(() -> intake.stop(), intake));

    addCommands(fullAuto); 
  }
}
