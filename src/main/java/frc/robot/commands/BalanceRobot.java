package frc.robot.commands;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

/**
 * Sets the robot to a target distance from an AprilTag. Used for balancing.
 */
public class BalanceRobot extends CommandBase {

  SwerveDrive swerveDrive;
  PhotonCamera camera;
  double prevDistance = 0.0;

  double targetDistance;

  public BalanceRobot(SwerveDrive _swerveDrive, double _targetDistance) {
    targetDistance = _targetDistance;
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
    prevDistance = distance;

    PIDController pidApriltag = new PIDController(1, 0, 0);

    double powerApriltag = pidApriltag.calculate(distance, targetDistance);
    powerApriltag = powerApriltag / 10 * 4;
    pidApriltag.setTolerance(0.1);

    System.out.println("Distance:" + distance); // Useful for Calibration

    if (pidApriltag.atSetpoint()) {
      swerveDrive.setWheelsToXFormation();
    } else {
      swerveDrive.drive(-powerApriltag, 0, 0, true);
    }

    pidApriltag.close();
  }

  @Override
  public void end(boolean interrupted) {
    swerveDrive.drive(0, 0, 0, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Gets the distance of the robot from an AprilTag
   * 
   * @return distance from AprilTag
   */
  public double getDistance() {
    PhotonPipelineResult result = camera.getLatestResult();
    List<PhotonTrackedTarget> targets = result.getTargets();

    for (PhotonTrackedTarget target : targets) {
      if (target.getFiducialId() == 7 || target.getFiducialId() == 2) {
        return target.getBestCameraToTarget().getX();
      }
    }
    return prevDistance;
  }

}
