package frc.commons;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShootingInterpolator {
    private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap shooterVelocityMap = new InterpolatingDoubleTreeMap();

    static {
        // distance (meters) -> hood angle (degrees)

        hoodAngleMap.put(3.02, 0.0);
        shooterVelocityMap.put(3.02,16.65);
        hoodAngleMap.put(2.392, 0.0);
        shooterVelocityMap.put(2.392, 14.61);
        hoodAngleMap.put(3.75, 1.0);
        shooterVelocityMap.put(3.75, 17.0 + 0.4);
        hoodAngleMap.put(3.479, 1.0);
        shooterVelocityMap.put(3.479, 17.0 + 0.2);
        hoodAngleMap.put(1.54, 0.0);
        shooterVelocityMap.put(1.54, 13.55);
        hoodAngleMap.put(3.64, 0.0);
        shooterVelocityMap.put(3.64, 17.5 + 0.3);
        hoodAngleMap.put(2.29, 0.2);
        shooterVelocityMap.put(2.29, 15.49);
        hoodAngleMap.put(2.785, 0.0);
        shooterVelocityMap.put(2.785, 15.0);
        hoodAngleMap.put(4.967, 1.7);
        shooterVelocityMap.put(4.967, 18.0 + 0.5);
        hoodAngleMap.put(2.811, 0.0);
        shooterVelocityMap.put(2.811, 14.9);

        /*hoodAngleMap.put(1.812, 0.0);
        shooterVelocityMap.put(1.812,12.4);
        hoodAngleMap.put(2.07, 0.0);
        shooterVelocityMap.put(2.07, 12.8);
        hoodAngleMap.put(2.35, 0.0);
        shooterVelocityMap.put(2.35, 13.4);
        hoodAngleMap.put(2.521, 0.0);
        shooterVelocityMap.put(2.521, 13.75);
        hoodAngleMap.put(2.755, 0.0);
        shooterVelocityMap.put(2.755, 14.45);
        hoodAngleMap.put(3.16, 0.0);
        shooterVelocityMap.put(3.16, 15.2);
        hoodAngleMap.put(3.56, 0.2);
        shooterVelocityMap.put(3.56, 15.8);
        hoodAngleMap.put(4.1, 0.0);
        shooterVelocityMap.put(4.1, 17.0);
        hoodAngleMap.put(3.83, 0.2);
        shooterVelocityMap.put(3.83, 16.65);
        hoodAngleMap.put(2.86, 0.0);
        shooterVelocityMap.put(2.86, 15.3);*/



        // distance (meters) -> shooter velocity
        /*shooterVelocityMap.put(1.406, 13.8);
        shooterVelocityMap.put(1.756, 13.75);
        shooterVelocityMap.put(1.888, 14.0);
        shooterVelocityMap.put(1.992, 14.3);
        shooterVelocityMap.put(2.411, 14.8);
        shooterVelocityMap.put(2.72,  15.4);
        shooterVelocityMap.put(2.96,  16.0);
        shooterVelocityMap.put(3.33,  17.1);
        shooterVelocityMap.put(3.525, 17.6);
        shooterVelocityMap.put(4.21,  19.3);*/

        /*shooterVelocityMap.put(1.72,13.8);
        shooterVelocityMap.put(2.06,14.2);
        shooterVelocityMap.put(2.2,14.5);
        shooterVelocityMap.put(2.46,14.6);
        shooterVelocityMap.put(2.551,14.8);
        shooterVelocityMap.put(2.82,15.2);
        shooterVelocityMap.put(3.03,15.8);
        shooterVelocityMap.put(3.366,16.4);
        shooterVelocityMap.put(4.0,17.9);*/


    }

    public static double getHoodAngle(double distanceMeters) {
        return hoodAngleMap.get(distanceMeters);
    }

    public static double getShooterVelocity(double distanceMeters) {
        return shooterVelocityMap.get(distanceMeters);
    }
}