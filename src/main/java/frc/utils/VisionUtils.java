package frc.utils;

import edu.wpi.first.math.util.Units;

public class VisionUtils {


    public static double cameraHeightInches = 14;
    public static double cameraAngleDeg = 25;


    private static double cameraHeightMeters = Units.inchesToMeters(cameraHeightInches);
    private static double cameraAngleRad = Units.degreesToRadians(cameraAngleDeg);


    private static double targetHeight = 1;

    private static double heightDiff = targetHeight - cameraHeightMeters;


    public static double getDist(double angleRad){
        double theta = angleRad + cameraAngleRad;
        return heightDiff/Math.tan(theta);
    }

}
