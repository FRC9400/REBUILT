package frc.commons;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShootingInterpolator {
    private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap shooterVelocityMap = new InterpolatingDoubleTreeMap();

    static {
        // distance (meters) -> hood angle (degrees)
        /*hoodAngleMap.put(1.406, 0.0);
        hoodAngleMap.put(1.756, 0.0);
        hoodAngleMap.put(1.888, 0.0);
        hoodAngleMap.put(1.992, 0.0);
        hoodAngleMap.put(2.411, 1.4);
        hoodAngleMap.put(2.72,  1.8);
        hoodAngleMap.put(2.96,  2.0);
        hoodAngleMap.put(3.33,  2.15);
        hoodAngleMap.put(3.525, 2.3);
        hoodAngleMap.put(4.21,  2.8);*/

        hoodAngleMap.put(1.72, 0.0);
        hoodAngleMap.put(2.06,0.4);
        hoodAngleMap.put(2.2,0.6);
        hoodAngleMap.put(2.46,0.8);
        hoodAngleMap.put(2.551,1.0);
        hoodAngleMap.put(2.82,1.2);
        hoodAngleMap.put(3.03,1.4);
        hoodAngleMap.put(3.366,1.8);
        hoodAngleMap.put(4.0,2.4);

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

        shooterVelocityMap.put(1.72,13.8);
        shooterVelocityMap.put(2.06,14.2);
        shooterVelocityMap.put(2.2,14.5);
        shooterVelocityMap.put(2.46,14.6);
        shooterVelocityMap.put(2.551,14.8);
        shooterVelocityMap.put(2.82,15.2);
        shooterVelocityMap.put(3.03,15.8);
        shooterVelocityMap.put(3.366,16.4);
        shooterVelocityMap.put(4.0,17.9);
    }

    public static double getHoodAngle(double distanceMeters) {
        return hoodAngleMap.get(distanceMeters);
    }

    public static double getShooterVelocity(double distanceMeters) {
        return shooterVelocityMap.get(distanceMeters);
    }
}