package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;


public class RunIntake extends CommandBase {

    public RunIntake() {
        addRequirements(RobotContainer.intake);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        RobotContainer.intake.intakeSpinIn();
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.intake.intakeStop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
