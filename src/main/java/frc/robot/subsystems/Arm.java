// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANID;
import static frc.robot.Constants.ArmConstants.*;

public class Arm extends SubsystemBase {

  CANSparkMax leftMotor;
  CANSparkMax rightMotor;

  /** Creates a new Arm. */
  public Arm() {

    leftMotor = new CANSparkMax(CANID.ARM_SPARKMAX_LEADER, MotorType.kBrushless);
    rightMotor = new CANSparkMax(CANID.ARM_SPARKMAX_FOLLOWER, MotorType.kBrushless);

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    leftMotor.setIdleMode(IdleMode.kCoast);
    rightMotor.setIdleMode(IdleMode.kCoast);

    leftMotor.setInverted(false);
    rightMotor.setInverted(false);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getPos();
  }

  public void move(double speed) {
    leftMotor.set(speed * 0.5);
    rightMotor.set(-speed * 0.5);
  }

  public double[] getPos() {
    System.out.println("left arm motor: " + leftMotor.getEncoder().getPosition());
    System.out.println("right arm motor: " + rightMotor.getEncoder().getPosition());
    return new double[] {
        leftMotor.getEncoder().getPosition(),
        rightMotor.getEncoder().getPosition()
    };
  }

}
