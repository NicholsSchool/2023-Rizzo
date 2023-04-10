package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class BalanceRobot extends CommandBase {

  SwerveDrive swerveDrive;
  boolean isFacingChargeStation = false;

  boolean isAtStopPoint = false;
  PhotonCamera camera;

  private static final double ANGLE_OF_INCLINE = 12.5;
  private static final double ANGLE_OF_DECLINE = -6.25;

  private static double ANGLE_OF_INCLINE_SPEED = 0.7;
  private static double ANGLE_OF_DECLINE_SPEED = -0.33;

  private static double APRILTAG_TO_CHARGE_STATION_METERS = 2.74; 

  public BalanceRobot(SwerveDrive _swerveDrive, boolean _isFacingChargeStation) {
    swerveDrive = _swerveDrive;
    isFacingChargeStation = _isFacingChargeStation;
    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    camera = new PhotonCamera( "Microsoft_LifeCam_HD-3000" );
    System.out.println("!!!!!!!!!!!! STARTING BALANCE ROUTINE !!!!!!!!!!!!!!");
  }


  /**
   * Gets the distance of the robot from an apriltag
   * 
   * @return the distance of the robot from the apriltag
   */
  public double getDistance()
  {
    PhotonPipelineResult result = camera.getLatestResult(); 
    if( result.hasTargets() )
    {
      PhotonTrackedTarget target = result.getBestTarget();
      double x = target.getBestCameraToTarget().getX(); //Not sure if it's x
      double y = target.getBestCameraToTarget().getY(); //Not sure if it's y
      return x; 
    }
    return 0.0; 
  }



  @Override
  public void execute() {
    double distance = getDistance();

    PIDController pidApriltag = new PIDController( 1, 0, 0); 

    double powerApriltag = pidApriltag.calculate( distance, APRILTAG_TO_CHARGE_STATION_METERS);

    if( distance < APRILTAG_TO_CHARGE_STATION_METERS )
    {
      swerveDrive.drive( -powerApriltag, 0, 0, true);
    }
    else if( distance > APRILTAG_TO_CHARGE_STATION_METERS )
    {
      swerveDrive.drive( powerApriltag, 0, 0, true);
    }
    else
    {
      pidApriltag.close();
    }
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
