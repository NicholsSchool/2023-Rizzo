package frc.robot.commands;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class BalanceRobot extends CommandBase {

  SwerveDrive swerveDrive;
  PhotonCamera camera;

  double prevDistance = 0.0;
  boolean firstPassCompleted = false; 
  boolean stop = false; 

  double DISTANCE = 0; 

  


  public BalanceRobot(SwerveDrive _swerveDrive, double distance) {
    DISTANCE = distance; 
    swerveDrive = _swerveDrive;
    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    camera = new PhotonCamera( "Microsoft_LifeCam_HD-3000" );
  }


  /**
   * Gets the distance of the robot from an apriltag
   * 
   * @return the distance of the robot from the apriltag. If no Apriltag found returns prev distance
   */
  public double getDistance()
  {
    PhotonPipelineResult result = camera.getLatestResult(); 
    List<PhotonTrackedTarget> targets = result.getTargets();

    for( PhotonTrackedTarget target: targets )
    {
      if( target.getFiducialId() == 7 || target.getFiducialId() == 2 )
      {
        double x = target.getBestCameraToTarget().getX(); //Not sure if it's x
        prevDistance = x;
        return x; 
      }
    }
    return prevDistance; 
  }



  @Override
  public void execute() {
    double distance = getDistance();

      PIDController pidApriltag = new PIDController( 1, 0, 0); 

      double powerApriltag = pidApriltag.calculate( distance, DISTANCE);
      powerApriltag = powerApriltag / 10 * 4; 
      pidApriltag.setTolerance( 0.1 );

      System.out.println("Distnace:" + distance); //Useful for Calibration

      if( pidApriltag.atSetpoint() )
      {
        pidApriltag.close();
        swerveDrive.setWheelsToXFormation();
      }
      else if( distance < DISTANCE && !pidApriltag.atSetpoint() )
      {
        swerveDrive.drive( -powerApriltag, 0, 0, true);
      }
      else if( distance > DISTANCE && !pidApriltag.atSetpoint() )
      {
        swerveDrive.drive( -powerApriltag, 0, 0, true);
      }

  }

  @Override
  public void end(boolean interrupted) {
    swerveDrive.drive( 0, 0, 0, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
