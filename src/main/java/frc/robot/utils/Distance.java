package frc.robot.utils;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import static frc.robot.RobotContainer.*;

public class Distance {

  /**
   * Returns a double[] containing [0] the angle to and [1] distance to the
   * nearest game element.
   * 
   * @return a double[] containing [0] the angle to and [1] distance to the
   *         nearest game element.
   */
  public static double[] getPolar() {
    String pieceID = pieceIDs.getStringTopic("piece").subscribe("!!!").get();

    double pixelsWide = pieceIDs.getIntegerTopic("xMax").subscribe(-0).get()
        - pieceIDs.getIntegerTopic("xMin").subscribe(-0).get();
    double pixelsTall = pieceIDs.getIntegerTopic("yMax").subscribe(-0).get()
        - pieceIDs.getIntegerTopic("yMin").subscribe(-0).get();

    double angle = Constants.Webcam.RADIANS_PER_PIXEL * (pieceIDs.getIntegerTopic("xMax").subscribe(-0).get()
        + RobotContainer.pieceIDs.getIntegerTopic("xMin").subscribe(-0).get()) / 2;
    double distance = -0.0;

    if (pieceID.equals("Cone")) {
      boolean toppled = pixelsWide > pixelsTall ? true : false;
      double theta = Constants.Webcam.RADIANS_PER_PIXEL * pixelsWide;
      distance = (toppled ? Constants.Cone.GREATER_DIM : Constants.Cone.LESSER_DIM) / 2 / Math.tan(theta);
    } else if (pieceID.equals("Cube")) {
      double theta = Constants.Webcam.RADIANS_PER_PIXEL * pixelsWide;
      distance = Constants.Cube.SIZE / 2 / Math.tan(theta);
    }
    return new double[] { angle, distance };
  }
}
