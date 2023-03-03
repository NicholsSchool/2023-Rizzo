// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.PID;

import frc.robot.Constants.ArmConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private WPI_TalonFX raiser;
  /** Creates a new Arm. */
  public Arm() {
    raiser = new WPI_TalonFX(ArmConstants.RAISER_ID);
    raiser.configFactoryDefault();
    raiser.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    raiser.setInverted(true); // check if change needed
    raiser.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
