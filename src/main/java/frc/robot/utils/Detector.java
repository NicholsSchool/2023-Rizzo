package frc.robot.utils;

import static frc.robot.RobotContainer.gamePieceIDs; // ???
import static frc.robot.RobotContainer.gamePieceCoordinates;

public class Detector {

  /**
   * Gets the x distance to a game piece
   * 
   * @return the x distance to a game piece
   */
  public int getXDist() {
    return (int) (gamePieceCoordinates.getDoubleTopic("X").subscribe(-0).get());
  }

  /**
   * Gets the y distance to a game piece
   * 
   * @return the y distance to a game piece
   */
  public int getYDist() {
    return (int) (gamePieceCoordinates.getDoubleTopic("Y").subscribe(-0).get());
  }

  /**
   * Gets the game piece's angle relative to the robot
   * 
   * @return the angle of a game piece relative to the robot
   */
  public double getAngle() {
    return gamePieceCoordinates.getDoubleTopic("Angle").subscribe(-0).get();
  }

}
