package frc.robot.Constants;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;

public class shooterConstants {
    /* Inverts */
    public static final InvertedValue shooterInvert = InvertedValue.Clockwise_Positive;

    /* Current Limits */
    public static final double statorCurrentLimit = 50;

    /* Circumference */
    public static double wheelCircumferenceMeters = Units.inchesToMeters(4) * Math.PI; 
    
    /* Gear Ratios */
    public static double shooterGearRatio = 0.5;
    
    /* PID Constants */
     public static final double kP = 0.068419;
    public static final double kD = 0.0;
    public static final double kS = 0.16488;
    public static final double kV = 0.11167;
    public static final double kA = 0.0077173;

    /* Shooter Velocity (MPS) — tune these */
    public static final double shootVelocityMPS = 10.0;  // TODO: set shoot velocity
    public static final double shootRatio = 1.0;          // TODO: set top/bottom ratio

    /* Current threshold — when stator drops below this, algae has left */
    public static final double shootCurrentThresholdAmps = 10.0; // TODO: tune threshold
    
}
