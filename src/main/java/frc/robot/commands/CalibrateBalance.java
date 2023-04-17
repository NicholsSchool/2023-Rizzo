package frc.robot.commands;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class CalibrateBalance extends CommandBase {

  SwerveDrive swerveDrive;
  PhotonCamera camera;

  public CalibrateBalance(SwerveDrive _swerveDrive) {
    swerveDrive = _swerveDrive;
    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  }

  @Override
  public void execute() {
    double distance = getDistance();
    System.out.println("Distance: " + distance);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Gets the distance of the robot from an AprilTag.
   * 
   * @return the distance from AprilTag
   */
  public double getDistance() {
    PhotonPipelineResult result = camera.getLatestResult();
    List<PhotonTrackedTarget> targets = result.getTargets();

    for (PhotonTrackedTarget target : targets) {
      if (target.getFiducialId() == 7 || target.getFiducialId() == 2) {
        return target.getBestCameraToTarget().getX();
      }
    }
    return 0.0;
  }

}
