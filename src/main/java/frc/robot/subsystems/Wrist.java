package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import static frc.robot.Constants.WristConstants.*;

public class Wrist extends SubsystemBase {

  private Solenoid wristPiston;

  public Wrist() {
    // Wrist Pistons
    wristPiston = new Solenoid(PneumaticsModuleType.CTREPCM, WRIST_PISTON_SOLENOID_CHANNEL);

    // Set the starting position of the Wrist subsystem.
    store();
  }

  @Override
  public void periodic() {
  }

  // Wrist Pistons
  public void extend() {
    wristPiston.set(IS_EXTENDED);
  }

  public void store() {
    wristPiston.set(!IS_EXTENDED);
  }
}
