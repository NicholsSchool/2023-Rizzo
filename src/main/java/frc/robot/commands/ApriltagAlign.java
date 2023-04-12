package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class ApriltagAlign extends CommandBase {
  SwerveDrive swerveDrive;
  PhotonCamera camera;

  public ApriltagAlign(SwerveDrive _swerveDrive) {
    swerveDrive = _swerveDrive;
    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
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
      double ambiguity = target.getPoseAmbiguity();
      return new double[] { x, y, ambiguity };
    }
    return new double[] { 0.0, 0.0, 100 };
  }

  @Override
  public void execute() {

    
    double xDistance = getDistance()[0];
    double yDistance = getDistance()[1];

    PIDController xPID = new PIDController(1, 0, 0);
    PIDController yPID = new PIDController(1, 0, 0);

    xPID.setTolerance(0.05);
    yPID.setTolerance(0.05 );

    double xPower = xPID.calculate(xDistance, 0.8);
    xPower = xPower / 10 * 5;
    double yPower = yPID.calculate(yDistance, -0.1);
    yPower = yPower / 10 * 5;


    System.out.println("xPower:" + xPower);
    System.out.println("yPower:" + yPower);

    System.out.println("xsetpoint:" + !xPID.atSetpoint());
    System.out.println("ysetpoint:" + !yPID.atSetpoint());

    

    if( !xPID.atSetpoint() && !yPID.atSetpoint() )
    {
      swerveDrive.drive( -xPower, -yPower, 0, true ); 
    }
    else
    {
      swerveDrive.drive( 0.0, 0.0, 0.0, true ); 
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerveDrive.drive(0.0, 0.0, 0.0, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
