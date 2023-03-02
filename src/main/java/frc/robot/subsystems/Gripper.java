// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.GripperConstants;

public class Gripper extends SubsystemBase {
  private WPI_TalonFX spinner;
  private Solenoid pincher;

  /** Creates a new Gripper. */
  public Gripper() {
    // spinner
    spinner = new WPI_TalonFX(GripperConstants.SPINNER_ID);
    spinner.configFactoryDefault();
    spinner.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    spinner.setInverted(true); // check if change needed
    spinner.setNeutralMode(NeutralMode.Coast);

    // pincher
    pincher = new Solenoid(PneumaticsModuleType.CTREPCM, GripperConstants.PINCHER_SOLENOID_CHANNEL);
    pincher.set(!GripperConstants.EXTENDED);
  }

  public void grab(double speed) {
    spinner.set(speed);
  }

  public void release(double speed) {
    spinner.set(-speed);
  }

  public void widen() {
    if (pincher.get() == GripperConstants.EXTENDED)
      pincher.set(!GripperConstants.EXTENDED);
  }

  public void narrow() {
    if (pincher.get() != GripperConstants.EXTENDED)
      pincher.set(GripperConstants.EXTENDED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
