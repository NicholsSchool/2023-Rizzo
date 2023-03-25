package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANID;
import static frc.robot.Constants.GripperConstants.*;

public class Gripper extends SubsystemBase {

  private CANSparkMax spinner;
  private Solenoid pincher;
  public static boolean state;

  public Gripper() {

    // Spinner
    spinner = new CANSparkMax(CANID.GRIPPER_SPARKMAX, MotorType.kBrushless);
    spinner.restoreFactoryDefaults();
    spinner.setIdleMode(IdleMode.kBrake);
    spinner.setInverted(true);

    // Pincher
    pincher = new Solenoid(PneumaticsModuleType.CTREPCM, PINCHER_SOLENOID_CHANNEL);

    // Set the starting state of the gripper subsystem.
    openPincher();

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

  public void stop() {
    spinner.stopMotor();
  }

  public void openPincher() {
    pincher.set(true);
  }

  public void closePincher() {
    pincher.set(false);
  }

  /**
   * Spin the gripper motor directly with OI controller.
   * 
   * @param speed double between -1.0 and 1.0
   */
  public void spin(double speed) {
    spinner.set(speed);
  }

}
