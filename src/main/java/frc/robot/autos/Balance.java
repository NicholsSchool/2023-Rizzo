package frc.robot.autos;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class Balance extends SequentialCommandGroup {
  SwerveDrive swerveDrive;
  Intake intake;
  Uprighter uprighter;
  Gripper gripper;
  Arm arm;

  public Balance(SwerveDrive _swerveDrive, Intake _intake, Uprighter _uprighter, Gripper _gripper, Arm _arm) {

    swerveDrive = _swerveDrive;
    intake = _intake;
    uprighter = _uprighter;
    gripper = _gripper;
    arm = _arm;

    addRequirements(swerveDrive, intake, gripper, arm, uprighter);

    addCommands(new BalanceRobot(_swerveDrive, false));

  }
}
