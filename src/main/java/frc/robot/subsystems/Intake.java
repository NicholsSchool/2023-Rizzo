// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private CANSparkMax motorLeft;
  private CANSparkMax motorRight;
  private Solenoid lifterLeft;
  private Solenoid lifterRight;
  // private Solenoid pincherLeft;
  // private Solenoid pincherRight;

  /** Creates a new Intake. */
  public Intake() {
    // Motor
    motorLeft = new CANSparkMax( IntakeConstants.LEFT_MOTOR_ID, MotorType.kBrushless );
    motorRight = new CANSparkMax( IntakeConstants.RIGHT_MOTOR_ID, MotorType.kBrushless );

    motorLeft.restoreFactoryDefaults();
    motorRight.restoreFactoryDefaults();

    motorLeft.setIdleMode( IdleMode.kBrake );
    motorRight.setIdleMode( IdleMode.kBrake );

    motorLeft.setInverted( false );
    motorRight.setInverted( false ); //maybe change this?

    //Pincher
    // pincherLeft = new Solenoid( PneumaticsModuleType.CTREPCM,IntakeConstants.PINCHER_LEFT_SOLENOID_CHANNEL );
    // pincherRight = new Solenoid( PneumaticsModuleType.CTREPCM, IntakeConstants.PINCHER_RIGHT_SOLENOID_CHANNEL );

    // pincherLeft.set( !IntakeConstants.EXTENDED );
    // pincherRight.set( !IntakeConstants.EXTENDED );

    //Lifter
    lifterLeft = new Solenoid( PneumaticsModuleType.CTREPCM, IntakeConstants.LIFTER_LEFT_SOLENOID_CHANNEL );
    lifterRight = new Solenoid( PneumaticsModuleType.CTREPCM, IntakeConstants.LIFTER_RIGHT_SOLENOID_CHANNEL );

    lifterLeft.set( !IntakeConstants.EXTENDED );
    lifterRight.set( !IntakeConstants.EXTENDED );
  }

  public void in( double speed ) {
        //Flip the negative if wrong direction
        motorLeft.set( speed );
        motorRight.set( -speed );
    }

    public void out( double speed ) {
        //Flip the negative if wrong direction
        motorLeft.set( -speed );
        motorRight.set( speed );
    }

    public void stop() {
        motorLeft.stopMotor();
        motorRight.stopMotor();
    }

    public void raise() {
      if( lifterLeft.get() == IntakeConstants.EXTENDED )
      {
        lifterLeft.set( !IntakeConstants.EXTENDED );
        lifterRight.set( !IntakeConstants.EXTENDED );
      }
    }

    public void lower() {
      if( lifterLeft.get() != IntakeConstants.EXTENDED )
      {
        lifterLeft.set( IntakeConstants.EXTENDED );
        lifterRight.set( IntakeConstants.EXTENDED );
      }
    }

    // public void narrow() {
    //     pincherLeft.
    //     pincherRight.
    // }

    // public void widen() {
    //     pincherLeft.
    //     pincherRight.
    // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
