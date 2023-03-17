package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.CANID;
import static frc.robot.Constants.GripperConstants.*;

public class Gripper extends SubsystemBase {

  private CANSparkMax spinner;
  private Solenoid pincher;
  public static boolean state;

  public Gripper() {

    state = PINCHER_CLOSED;

    // Spinner - Brake mode enabled to hold a game piece during autonomous.
    spinner = new CANSparkMax(CANID.GRIPPER_SPARKMAX, MotorType.kBrushless);
    spinner.restoreFactoryDefaults();
    spinner.setIdleMode(IdleMode.kBrake);
    spinner.setInverted(true);

    // Pincher - Set to closed to hold a game piece during autonomous.
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

  public void stopSpinner() {
    spinner.stopMotor();
  }

  public void openPincher() {
    pincher.set(!PINCHER_CLOSED);
    state = !PINCHER_CLOSED;
  }

  public void closePincher() {
    pincher.set(PINCHER_CLOSED);
    state = PINCHER_CLOSED;
  }

  /**
   * Changes gripper piston for picking up cones or cubes
   */
  public void setGripperState() {
    RobotContainer.readyForCube = !RobotContainer.readyForCube;
  }

  public boolean getGripperState() {
    return RobotContainer.readyForCube;
  }

  public void gripPiece() {
    if (getGripperState()) {
      closePincher();
    }
  }

}
