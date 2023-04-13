// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class MLPickup extends CommandBase {
  private static final int WIDTH = 160; 
  private static final int HEIGHT = 120;

  SwerveDrive swerveDrive;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();

  NetworkTable pieces = inst.getTable("Vision");
  NetworkTable names = inst.getTable("Piece");


  StringSubscriber name = names.getStringTopic("piece").subscribe("None");


  IntegerSubscriber ymin = pieces.getIntegerTopic("yMin").subscribe(0);
  IntegerSubscriber xmin = pieces.getIntegerTopic("xMin").subscribe(0);
  IntegerSubscriber ymax = pieces.getIntegerTopic("yMax").subscribe(0);
  IntegerSubscriber xmax = pieces.getIntegerTopic("xMax").subscribe(0);
  IntegerSubscriber distance = pieces.getIntegerTopic("Distance").subscribe(0);


  public MLPickup(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive; 

    addRequirements( this.swerveDrive );
  }

  @Override
  public void initialize() {
  }

  /**
   * Gets the center x coordinate of the detected game piece
   * 
   * @return the center x coordinate of the deteced game piece
   */
  public int getXCenter()
  {
    long xMax = xmax.get(); 
    long xMin = xmin.get(); 

    long width = xMax - xMin;
    long xCenter = xMin + ( width / 2 ); 

    return (int)(xCenter); 
  }


  /**
   * Gets the center y coordinate of the detected game piece
   * 
   * @return the center y coordinate of the deteced game piece
   */
  public int getYCenter()
  {
    long yMax = ymax.get(); 
    long yMin = ymin.get(); 

    long height = yMax - yMin;
    long yCenter = yMin + ( height / 2 ); 

    return (int)(yCenter); 
  }

  /**
   * Gets the distance from the robot to a detected game piece in meters
   * 
   * @return a game pieces' distance from the robot
   */
  public double getDistance()
  {
    long d = distance.get(); 
    return ( double )( d ); 
  }

  /**
   * Gets the name of a detected game piece
   * 
   * @return the name of a detected game piece
   */
  public String getName()
  {
    return name.get(); 
  }



  @Override
  public void execute() {
    String name = getName();
    int xCenter = getXCenter();
    int yCenter = getYCenter();
    if( name.equals("Cube") )
    {

      PIDController xPID = new PIDController(1, 0, 0);
      PIDController yPID = new PIDController(1, 0, 0);

      xPID.setTolerance(.05);
      yPID.setTolerance(.05);

      double xPower = xPID.calculate(xCenter, WIDTH / 2 );
      xPower = xPower / 1000 * 6;
      double yPower = yPID.calculate(yCenter, HEIGHT);
      yPower = yPower / 1000 * 6;

      if( !xPID.atSetpoint() && !yPID.atSetpoint() )
      swerveDrive.drive( yPower, xPower, 0, false);
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
