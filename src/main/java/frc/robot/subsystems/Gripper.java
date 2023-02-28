// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.GripperConstants;

public class Gripper extends SubsystemBase {
  private WPI_TalonFX rotator;

  /** Creates a new Gripper. */
  public Gripper() {
    rotator = new WPI_TalonFX(GripperConstants.ROTATOR_ID);

    rotator.configFactoryDefault();

    rotator.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    rotator.setInverted(true); // check if change needed

    rotator.setNeutralMode(NeutralMode.Coast);
  }

  public void goTo( int position ) {
    //PID controller needed
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
