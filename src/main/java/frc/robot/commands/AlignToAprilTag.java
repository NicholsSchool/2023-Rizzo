package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

/**
 * Aligns the robot to the nearest AprilTag.
 */
public class AlignToAprilTag extends CommandBase {

  SwerveDrive swerveDrive;
  PhotonCamera camera;

  public AlignToAprilTag(SwerveDrive _swerveDrive) {
    swerveDrive = _swerveDrive;
    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  }

  @Override
  public void execute() {

    double[] distance = getDistance();

    double xDistance = distance[0];
    double yDistance = distance[1];

    PIDController xPID = new PIDController(1, 0, 0);
    PIDController yPID = new PIDController(1, 0, 0);

    xPID.setTolerance(0.05);
    yPID.setTolerance(0.05);

    double xPower = xPID.calculate(xDistance, 0.8);
    xPower = xPower / 10 * 5;

    double yPower = yPID.calculate(yDistance, -0.1);
    yPower = yPower / 10 * 5;

    if (!xPID.atSetpoint() && !yPID.atSetpoint()) {
      swerveDrive.drive(-xPower, -yPower, 0.0, true);
    } else {
      swerveDrive.drive(0.0, 0.0, 0.0, true);
    }

    xPID.close();
    yPID.close();
  }

  @Override
  public void end(boolean interrupted) {
    swerveDrive.drive(0.0, 0.0, 0.0, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Gets the distance of the robot from an apriltag
   * 
   * @return the distance of the robot from the apriltag
   */
  public double[] getDistance() {
    PhotonPipelineResult result = camera.getLatestResult();
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      double x = target.getBestCameraToTarget().getX();
      double y = target.getBestCameraToTarget().getY();
      double rot = target.getBestCameraToTarget().getRotation().toRotation2d().getDegrees();
      double ambiguity = target.getPoseAmbiguity();
      return new double[] { x, y, ambiguity, rot };
    }
    return new double[] { 0.0, 0.0, 100, 180 };
  }

}
