package frc.robot.Constants;

import com.ctre.phoenix6.signals.InvertedValue;

public class intakeConstants {
    /* Motor Inverts */
    public static final InvertedValue pivotInvert = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue intakeInvert = InvertedValue.Clockwise_Positive;

    /* Current Limits */
    public static final double pivotCurrentLimit = 40; // lowered from 70, pivot doesn't need much
    public static final double pivotStatorCurrentLimit = 40; // stator limit for pivot
    public static final double intakeCurrentLimit = 80; // lowered from 140

    /* Gear Ratios */
    public static final double gearRatio = 33.333;

    /* Min and Max */
    public static final double minDeg = 0;
    public static final double maxDeg = 120;
}