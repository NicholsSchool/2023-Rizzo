package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CBArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Uprighter;

public class AutoTest extends SequentialCommandGroup {
  /** Creates a new AutoTest. */
  public AutoTest( SwerveDrive swerveDrive, CBArm arm, Intake intake, Uprighter uprighter, Gripper gripper ) {
  
    addRequirements(swerveDrive, arm, intake, gripper );
    addCommands(
      new IntakeDeploy( intake, uprighter, gripper ).withTimeout(1)
      // ,
      // new RunCommand( () -> intake.spinIn() ).withTimeout( 0.4 ),
      // new IntakeRetract(intake, uprighter, gripper)
    );
  }
}
