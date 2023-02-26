// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.UprighterConstants;

public class Uprighter extends SubsystemBase {
  private CANSparkMax motor;

  /** Creates a new Uprighter. */
  public Uprighter() {
    // Motor
    motor = new CANSparkMax(UprighterConstants.MOTOR_ID, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(false); // check if clockwise is incorrect without inversion
  }

  public void clockwise(double speed) {
    motor.set(speed);
  }

  public void counterClockwise(double speed) {
    motor.set(-speed);
  }

  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
