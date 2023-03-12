package frc.robot.utils;

import frc.robot.RobotContainer;
import static frc.robot.Constants.*;
import static frc.robot.RobotContainer.gamePieceIDs;;

public class Detector {

  /**
   * The getPolar() method returns a double array of two elements, representing
   * the polar coordinates of a detected game piece.
   * 
   * The getPolar() method calculates the angle of the game piece using the
   * formula H_RADIANS_PER_PIXEL * (xMax + xMin) / 2, where H_RADIANS_PER_PIXEL is
   * a constant value and xMax and xMin are the maximum and minimum x values of
   * the game piece. This calculation finds the horizontal angle of the game piece
   * relative to the camera.
   * 
   * The getPolar() method then calculates the distance to the game piece using
   * different formulas depending on the type of game piece detected.
   * 
   * @return double[] = [0] angle and [1] distance to the nearest game piece.
   */
  public static double[] getPolar() {

    String gamePieceID = gamePieceIDs.getStringTopic("piece").subscribe("!!!").get();

    double pixelsWide = gamePieceIDs.getIntegerTopic("xMax").subscribe(-0).get()
        - gamePieceIDs.getIntegerTopic("xMin").subscribe(-0).get();

    double pixelsTall = gamePieceIDs.getIntegerTopic("yMax").subscribe(-0).get()
        - gamePieceIDs.getIntegerTopic("yMin").subscribe(-0).get();

    double angle = WebcamConstants.H_RADIANS_PER_PIXEL * (gamePieceIDs.getIntegerTopic("xMax").subscribe(-0).get()
        + RobotContainer.gamePieceIDs.getIntegerTopic("xMin").subscribe(-0).get()) / 2;

    double distance = -0.0;

    if (gamePieceID.equals("Cone")) {
      boolean toppled = pixelsWide > pixelsTall ? true : false;
      double theta = WebcamConstants.H_RADIANS_PER_PIXEL * pixelsWide;
      distance = (toppled ? ConeConstants.GREATER_DIM : ConeConstants.LESSER_DIM) / 2 / Math.tan(theta);
    } else if (gamePieceID.equals("Cube")) {
      double theta = WebcamConstants.H_RADIANS_PER_PIXEL * pixelsWide;
      distance = CubeConstants.SIZE / 2 / Math.tan(theta);
    }

    return new double[] { angle, distance };
  }
}
