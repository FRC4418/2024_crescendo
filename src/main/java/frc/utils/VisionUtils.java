package frc.utils;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionUtils {


    public static double cameraHeightInches = 16;
    public static double cameraAngleDeg = 22;


    private static double cameraHeightMeters = Units.inchesToMeters(cameraHeightInches);
    private static double cameraAngleRad = Units.degreesToRadians(cameraAngleDeg);


    private static double targetHeightMeters = 1.44;

    private static double heightDiff = targetHeightMeters - cameraHeightMeters;


    public static double getDist(double angleDeg){
        double theta = Units.degreesToRadians(angleDeg) + cameraAngleRad;
        return heightDiff/Math.tan(theta);
    }

    public static PhotonTrackedTarget getId(PhotonPipelineResult result, int id){
        for (var target : result.getTargets()){
            if(target.getFiducialId() == id){
                return target;
            }
        }
        return null;
    }

    public static PhotonTrackedTarget getId(PhotonPipelineResult camResult, int id1, int id2){
        var result1 = getId(camResult, id1);
        var result2 = getId(camResult, id2);

        if(result1 != null) return result1;
        if(result2 != null) return result2;
        return null;
    }
}
