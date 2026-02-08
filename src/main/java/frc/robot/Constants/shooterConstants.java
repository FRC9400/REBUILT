package frc.robot.Constants;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;

public class shooterConstants {
    /* Inverts */
    public static final InvertedValue shooterInvert = InvertedValue.Clockwise_Positive;

    /* Current Limits */
    public static final double shooterCurrentLimit = 70;

    /* Circumference */
    public static double wheelCircumferenceMeters = Units.inchesToMeters(4) * Math.PI; 
    
    /* Gear Ratios */
    public static double shooterGearRatio = 1;
    
}
