// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class CBArm extends SubsystemBase {

  CANSparkMax lMotor;
  CANSparkMax rMotor;
  RelativeEncoder lEncoder;
  RelativeEncoder rEncoder;
  SparkMaxPIDController lPID;
  SparkMaxPIDController rPID;

  /** Creates a new CBArm. */
  public CBArm() {
    lMotor = new CANSparkMax(25, MotorType.kBrushless);
    rMotor = new CANSparkMax(26, MotorType.kBrushless);

    lMotor.restoreFactoryDefaults();
    rMotor.restoreFactoryDefaults();

    lMotor.setIdleMode(IdleMode.kCoast);
    rMotor.setIdleMode(IdleMode.kCoast);

    lMotor.setInverted(false);
    rMotor.setInverted(false);

    lEncoder = lMotor.getEncoder();
    rEncoder = rMotor.getEncoder();

    lEncoder.setPosition(0.0);
    rEncoder.setPosition(0.0);

    lPID = lMotor.getPIDController();
    rPID = rMotor.getPIDController();

    lPID.setFeedbackDevice(lEncoder);
    rPID.setFeedbackDevice(rEncoder);

    lPID.setP(0.33);
    lPID.setI(0.15);
    lPID.setD(0.0);
    lPID.setIZone(0);
    lPID.setFF(0.00035);
    lPID.setOutputRange(-1.0, 1.0);

    rPID.setP(0.33);
    rPID.setI(0.15);
    rPID.setD(0.0);
    rPID.setIZone(0);
    rPID.setFF(0.00035);
    rPID.setOutputRange(-1.0, 1.0);

  }

  @Override
  public void periodic() {
    System.out.println(rEncoder.getPosition());
  }

  public void move(double speed) {
    if (speed < 0 && rEncoder.getPosition() < ArmConstants.HOME_POSITION + 10.00 ||
        speed > 0 && rEncoder.getPosition() > ArmConstants.INTAKE_POSITION - 10.00) {
      lMotor.set(0.25 * -speed);
      rMotor.set(0.25 * speed);
    } else {
      lMotor.set(-speed);
      rMotor.set(speed);
    }
  }

  public void moveToPositionPID(double pos) {
    lPID.setReference(pos, CANSparkMax.ControlType.kPosition);
    rPID.setReference(-pos, CANSparkMax.ControlType.kPosition);
  }

  public void stop() {
    lMotor.stopMotor();
    rMotor.stopMotor();
  }

}
