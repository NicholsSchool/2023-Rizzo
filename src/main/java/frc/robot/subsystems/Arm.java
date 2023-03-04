package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  private WPI_TalonFX raiser;

  public Arm() {

    // arm
    raiser = new WPI_TalonFX(ArmConstants.RAISER_ID);
    raiser.configFactoryDefault();
    raiser.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    raiser.setInverted(true);
    raiser.setNeutralMode(NeutralMode.Coast);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
