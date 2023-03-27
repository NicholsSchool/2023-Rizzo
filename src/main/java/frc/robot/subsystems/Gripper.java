package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.CANID;
import static frc.robot.Constants.GripperConstants.*;

public class Gripper extends SubsystemBase {

  private CANSparkMax spinner;
  private Solenoid pincher;
  private DigitalInput limitSwitch;

  public Gripper() {

    // Spinner
    spinner = new CANSparkMax(CANID.GRIPPER_SPARKMAX, MotorType.kBrushless);
    spinner.restoreFactoryDefaults();
    spinner.setIdleMode(IdleMode.kBrake);
    spinner.setInverted(false);

    // Pincher
    pincher = new Solenoid(PneumaticsModuleType.CTREPCM, PINCHER_SOLENOID_CHANNEL);

    // Limit Switch
    limitSwitch = new DigitalInput(GRIPPER_LIMIT_SWITCH_DIO_CHANNEL);

    // Set the starting state of the gripper subsystem.
    open();

  }

  @Override
  public void periodic() {
    RobotContainer.gripperLimit.setBoolean(isPressed());
  }

  public void spinIn() {
    spinner.set(-GRIPPER_SPEED);
  }

  public void spinOut() {
    spinner.set(GRIPPER_SPEED);
  }

  public void stop() {
    spinner.stopMotor();
  }

  public void open() {
    pincher.set(false);
  }

  public void close() {
    pincher.set(true);
  }

  // Is limit switch pressed?
  public boolean isPressed() {
    return limitSwitch.get();
  }

  /**
   * Spin the gripper motor directly with OI controller.
   * 
   * @param speed double between -1.0 and 1.0
   */
  public void spin(double speed) {
    spinner.set(speed * GRIPPER_SPEED_OFFSET);
  }

}
