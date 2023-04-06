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

  private CANSparkMax gripperMotor;
  private Solenoid gripperPiston;
  private DigitalInput gripperLimitSwitch;
  private boolean isOpen = true;

  public Gripper() {

    // Gripper Motor
    gripperMotor = new CANSparkMax(CANID.GRIPPER_SPARKMAX, MotorType.kBrushless);
    gripperMotor.restoreFactoryDefaults();
    gripperMotor.setIdleMode(IdleMode.kBrake);
    gripperMotor.setInverted(false);

    // Gripper Piston/Pincher
    gripperPiston = new Solenoid(PneumaticsModuleType.CTREPCM, PINCHER_SOLENOID_CHANNEL);

    // Gripper Limit Switch
    gripperLimitSwitch = new DigitalInput(GRIPPER_LIMIT_SWITCH_DIO_CHANNEL);

    // Set the starting state of the gripper subsystem.
    open();

  }

  @Override
  public void periodic() {
    RobotContainer.gripperLimit.setBoolean(isPressed());
  }

  // Gripper Motors

  public void spin(double speed) {
    gripperMotor.set(speed * GRIPPER_SPEED_OFFSET);
  }

  public void spinIn() {
    gripperMotor.set(-GRIPPER_SPEED);
  }

  public void spinOut() {
    gripperMotor.set(GRIPPER_SPEED);
  }

  public void stop() {
    gripperMotor.stopMotor();
  }

  // Gripper Pistons

  public void open() {
    gripperPiston.set(false);
    isOpen = true;
  }

  public void close() {
    gripperPiston.set(true);
    isOpen = false;
  }

  public void toggle() {
    if (isOpen) {
      close();
    } else {
      open();
    }
  }

  // Gripper Limit Switch

  public boolean isPressed() {
    return gripperLimitSwitch.get();
  }

}
