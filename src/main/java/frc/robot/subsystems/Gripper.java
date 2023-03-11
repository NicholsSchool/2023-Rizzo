package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANID;
import static frc.robot.Constants.GripperConstants.*;

public class Gripper extends SubsystemBase {

  private WPI_TalonFX spinner;
  private Solenoid pincher;

  public Gripper() {

    // Spinner - Brake mode enabled to hold the game piece during autonomous.
    spinner = new WPI_TalonFX(CANID.GRIPPER_FALCON_FX);
    spinner.configFactoryDefault();
    spinner.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    spinner.setInverted(true);
    spinner.setNeutralMode(NeutralMode.Brake);

    // Pincher - Set to closed for holding a game piece during autonomous.
    pincher = new Solenoid(PneumaticsModuleType.CTREPCM, PINCHER_SOLENOID_CHANNEL);
    pincher.set(PINCHER_CLOSED);

  }

  @Override
  public void periodic() {
  }

  public void spinIn() {
    spinner.set(GRIPPER_SPEED);
  }

  public void spinOut() {
    spinner.set(-GRIPPER_SPEED);
  }

  public void openPincher() {
    pincher.set(!PINCHER_CLOSED);
  }

  public void closePincher() {
    pincher.set(PINCHER_CLOSED);
  }

  public double getSpinnerPosition() {
    return spinner.getSelectedSensorPosition();
  }

  public double getSpinnerVelocity() {
    return spinner.getSelectedSensorVelocity();
  }

}
