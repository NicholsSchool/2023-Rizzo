package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANID;
import frc.robot.Constants.GripperConstants;

public class Gripper extends SubsystemBase {

  private WPI_TalonFX spinner;
  private Solenoid pincher;

  public Gripper() {
    // spinner
    spinner = new WPI_TalonFX(CANID.GRIPPER_FALCON_FX);
    spinner.configFactoryDefault();
    spinner.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    spinner.setInverted(true);
    spinner.setNeutralMode(NeutralMode.Coast);
    // pincher
    pincher = new Solenoid(PneumaticsModuleType.CTREPCM, GripperConstants.PINCHER_SOLENOID_CHANNEL);
    pincher.set(!GripperConstants.GRIPPER_PISTON_EXTENDED);
  }

  public void grab(double speed) {
    close();
    spinner.set(speed);
  }

  public void release(double speed) {
    open();
    spinner.set(-speed);
  }

  public void open() {
    if (pincher.get() == GripperConstants.GRIPPER_PISTON_EXTENDED)
      pincher.set(!GripperConstants.GRIPPER_PISTON_EXTENDED);
  }

  public void close() {
    if (pincher.get() != GripperConstants.GRIPPER_PISTON_EXTENDED)
      pincher.set(GripperConstants.GRIPPER_PISTON_EXTENDED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
