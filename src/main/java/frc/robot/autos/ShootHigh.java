// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.OuttakeCube;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Uprighter;
import static frc.robot.Constants.IntakeConstants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootHigh extends SequentialCommandGroup {
  Intake intake;
  Uprighter uprighter;
  Gripper gripper;
  SwerveDrive swerveDrive;

  /** Creates a new ShootHigh. */
  public ShootHigh(SwerveDrive _swerveDrive, Intake _intake, Uprighter _uprighter, Gripper _gripper) {

    swerveDrive = _swerveDrive;
    intake = _intake;
    uprighter = _uprighter;
    gripper = _gripper;

    addCommands(new RunCommand(() -> uprighter.spinOut(), intake).withTimeout(1),
        new OuttakeCube(intake, uprighter, gripper, OUTTAKE_HIGH_SPEED).withTimeout(1),
        new InstantCommand(() -> swerveDrive.setGyroAngleAdjustment(180.0)));

  }
}
