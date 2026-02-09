package frc.robot.Constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class swerveConstants {
  public static final class moduleConstants {
    /* Inverts FL, FR, BL, BR */
    public static final InvertedValue[] driveMotorInverts = {
      InvertedValue.Clockwise_Positive,
      InvertedValue.CounterClockwise_Positive,
      InvertedValue.Clockwise_Positive,
      InvertedValue.CounterClockwise_Positive
    };
    public static final InvertedValue[] steerMotorInverts = {
      InvertedValue.Clockwise_Positive,
      InvertedValue.Clockwise_Positive,
      InvertedValue.Clockwise_Positive,
      InvertedValue.Clockwise_Positive
    };
    public static final SensorDirectionValue[] CANcoderInverts = {
      SensorDirectionValue.CounterClockwise_Positive,
      SensorDirectionValue.CounterClockwise_Positive,
      SensorDirectionValue.CounterClockwise_Positive,
      SensorDirectionValue.CounterClockwise_Positive
    };
    /* CANcoder Offset FL, FR, BL, BR */
    public static final double[] CANcoderOffsets = {0.121582, 0.182373, -0.254395, -0.296875};

    /* Gear Ratios */
    public static final double driveGearRatio = 6.12;
    public static final double steerGearRatio = 150.0 / 7.0;

    /* Max Speeds */
    public static final double maxSpeedMeterPerSecond = Units.feetToMeters(16);
    public static final double maxAngularVelocity = 4;

    /* Current Limits */
    public static final double driveStatorCurrentLimit = 80;
    public static final double steerStatorCurrentLimit = 50;

    /* Ramp Rate */
    public static final double rampRate = 0.02;

    /* Wheel Circumference */
    public static final double wheelCircumferenceMeters = Units.inchesToMeters(4) * Math.PI;
  }

  public static final class kinematicsConstants {
    /* Drivetrain Constants */
    public static final double wheelBase = Units.inchesToMeters(22.75);
    public static final double trackWidth = Units.inchesToMeters(20.75);

    /* Swerve Kinematics */
    public static final Translation2d FL = new Translation2d(wheelBase / 2.0, trackWidth / 2.0);
    public static final Translation2d FR = new Translation2d(wheelBase / 2.0, -trackWidth / 2.0);
    public static final Translation2d BL = new Translation2d(-wheelBase / 2.0, trackWidth / 2.0);
    public static final Translation2d BR = new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0);
  }
}
