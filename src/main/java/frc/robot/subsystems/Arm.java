package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANID;
import static frc.robot.Constants.ArmConstants.*;

public class Arm extends SubsystemBase {

  private CANSparkMax armMotor;
  private AnalogPotentiometer pot;

  public Arm() {

    // arm Motor
    armMotor = new CANSparkMax(CANID.ARM_SPARKMAX, MotorType.kBrushless);
    armMotor.restoreFactoryDefaults();
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setInverted(false); // check later
    armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    armMotor.setSoftLimit(SoftLimitDirection.kForward, (float) MAX_ARM_POS);
    armMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) MIN_ARM_POS);

    // potentiometer
    pot = new AnalogPotentiometer(POT_PORT, MAX_ARM_POS, START_ARM_POS);
    // TODO: conversion from volts to angle of the arm
  }

  @Override
  public void periodic() {
  }

  public double getPot() {
    return pot.get();
  }

  public void spin( double speed ) {
    armMotor.set(speed);
  }

  public void goToAngle(double desiredAngle) {
    double armAngle = getPot();
    armMotor.set( ARM_P * (desiredAngle - armAngle) );
  }

  public void stop() {
    armMotor.stopMotor();
  }
}