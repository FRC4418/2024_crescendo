// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class TrajectoryUtils {

    public static double gravity = -9.8;
    public static double rotationPointHeight = 0.4;
    public static double speakerHeight = 1.7;
    public static double initialVelocity = 50;
    public static double armLength = 0.485775; 
    public static double angleDifference = -25d; //-12d; // needs tuning
    public static double cameraShooterDist = Units.inchesToMeters(21.4);

    public static double getGoodShootingAngle(double distance){
        double angle = 0;

        distance += cameraShooterDist;

        while(true){
            if(angle > 89) return -1;

            double actuallDist = getDistanceFromAngle(angle);

            if(actuallDist < distance)  return angle;

            angle += 0.1;
        }
    }

    public static double getDistanceFromAngle(double angle){  //https://www.desmos.com/calculator/eae6ngbwa0
        double armAngle = -angle-angleDifference;
        //System.out.println("Arm angle: " + armAngle);
        double shootingHeight = Math.sin(toRad(armAngle)) * armLength + rotationPointHeight;
        //System.out.println("shooting height: " + shootingHeight);
        double xVelocity = Math.cos(toRad(angle)) * initialVelocity;
        double yVelocity = Math.sin(toRad(angle)) * initialVelocity;
        //System.out.println("xvel,yvel: " + xVelocity + "  " + yVelocity);

        double thingToSqrt = Math.pow(yVelocity,2) - 4 * gravity/2 * (shootingHeight - speakerHeight);

        //System.out.println("sqrt: " + thingToSqrt);

        if(thingToSqrt < 0){
            return 999;
        }

        return xVelocity * (
                (-yVelocity + Math.sqrt( thingToSqrt ))
                        /
                        (gravity)
        );
    }


    public static double toRad(double num){
        return Math.toRadians(num);
    }

    public static double toDegrees(double num){
        return Math.toDegrees(num);
    }

}
