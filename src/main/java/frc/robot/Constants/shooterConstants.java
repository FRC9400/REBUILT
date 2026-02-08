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
    
}
