package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;


public class RunIntake extends CommandBase {

    public RunIntake() {
        addRequirements(RobotContainer.intake, RobotContainer.uprighter);
    }

    @Override
    public void initialize()
    {
      RobotContainer.intake.lifterDown();
    }

    @Override
    public void execute() {
        RobotContainer.intake.intakeSpinIn();
        RobotContainer.uprighter.uprighterSpinIn();
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.intake.intakeStop();
        RobotContainer.uprighter.uprighterStop();
        RobotContainer.intake.lifterUp();

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
