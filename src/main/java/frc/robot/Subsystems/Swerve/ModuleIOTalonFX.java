package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import frc.commons.Conversions;
import frc.robot.Constants.swerveConstants;
import frc.robot.Constants.swerveConstants.moduleConstants;

public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveMotor;
  private final TalonFX steerMotor;
  private final CANcoder angleEncoder;
  private final double CANcoderOffset;
  private TalonFXConfiguration driveConfigs;
  private TalonFXConfiguration steerConfigs;
  private CANcoderConfiguration angleEncoderConfigs;

  private final StatusSignal<Angle> steerPos;
  private final StatusSignal<Angle> drivePos;
  private final StatusSignal<AngularVelocity> driveVelRPS;
  private final StatusSignal<Temperature> driveTemp;
  private final StatusSignal<Temperature> steerTemp;
  private final StatusSignal<Current> driveAmps;
  private final StatusSignal<Current> steerAmps;
  private final StatusSignal<Angle> absolutePositionRotations;

  private PositionVoltage steerRequest;
  private VelocityVoltage velocityVoltageRequest;
  private VoltageOut driveVoltageRequest;
  private VoltageOut steerVoltageRequest;

  public ModuleIOTalonFX(
      int driveID,
      int steerID,
      int CANcoderID,
      double CANcoderOffset,
      InvertedValue driveInvert,
      InvertedValue steerInvert,
      SensorDirectionValue CANcoderInvert) {
    driveMotor = new TalonFX(driveID, "canivore");
    steerMotor = new TalonFX(steerID, "canivore");
    angleEncoder = new CANcoder(CANcoderID, "canivore");
    this.CANcoderOffset = CANcoderOffset;
    driveConfigs = new TalonFXConfiguration();
    steerConfigs = new TalonFXConfiguration();
    angleEncoderConfigs = new CANcoderConfiguration();

    steerRequest = new PositionVoltage(0).withEnableFOC(true);
    velocityVoltageRequest = new VelocityVoltage(0).withEnableFOC(true);
    driveVoltageRequest = new VoltageOut(0).withEnableFOC(true);
    steerVoltageRequest = new VoltageOut(0).withEnableFOC(true);

    /* Drive Configs */
    var driveMotorOutputConfigs = driveConfigs.MotorOutput;
    driveMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    driveMotorOutputConfigs.Inverted = driveInvert;

    var driveFeedbackConfigs = driveConfigs.Feedback;
    driveFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    driveMotor.setPosition(0);

    var driveCurrentLimitConfigs = driveConfigs.CurrentLimits;
    driveCurrentLimitConfigs.StatorCurrentLimitEnable = true;
    driveCurrentLimitConfigs.StatorCurrentLimit =
        swerveConstants.moduleConstants.driveStatorCurrentLimit;

    var driveOpenLoopConfigs = driveConfigs.OpenLoopRamps;
    driveOpenLoopConfigs.VoltageOpenLoopRampPeriod = swerveConstants.moduleConstants.rampRate;

    var driveSlot0Configs = driveConfigs.Slot0;

    driveSlot0Configs.kP = 0.14;
    driveSlot0Configs.kI = 0;
    driveSlot0Configs.kD = 0;
    driveSlot0Configs.kS = 0.111;
    driveSlot0Configs.kV = 0.12;
    driveSlot0Configs.kA = 0.5;

    /* Steer Configs */
    var steerMotorOutputConfigs = steerConfigs.MotorOutput;
    steerMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    steerMotorOutputConfigs.Inverted = steerInvert;

    var steerFeedbackConfigs = steerConfigs.Feedback;
    steerFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    var steerSlot0Configs = steerConfigs.Slot0;
    steerSlot0Configs.kP = 6;
    steerSlot0Configs.kI = 0;
    steerSlot0Configs.kD = 0.002;
    steerSlot0Configs.kS = 0.24;
    steerSlot0Configs.kV = 0.001;
    steerSlot0Configs.kA = 0.16;

    var steerCurrentLimitConfigs = steerConfigs.CurrentLimits;
    steerCurrentLimitConfigs.StatorCurrentLimitEnable = true;
    steerCurrentLimitConfigs.StatorCurrentLimit =
        swerveConstants.moduleConstants.steerStatorCurrentLimit;

    /* CANCoder Configs */
    var angleEncoderConfig = new CANcoderConfiguration();
    angleEncoderConfig.MagnetSensor.SensorDirection =
        SensorDirectionValue.CounterClockwise_Positive;

    driveMotor.getConfigurator().apply(driveConfigs);
    steerMotor.getConfigurator().apply(steerConfigs);
    angleEncoder.getConfigurator().apply(angleEncoderConfigs);

    steerPos = steerMotor.getRotorPosition();
    drivePos = driveMotor.getRotorPosition();
    driveVelRPS = driveMotor.getRotorVelocity();
    driveTemp = driveMotor.getDeviceTemp();
    steerTemp = steerMotor.getDeviceTemp();
    driveAmps = driveMotor.getStatorCurrent();
    steerAmps = steerMotor.getStatorCurrent();
    absolutePositionRotations = angleEncoder.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, steerPos, drivePos, driveVelRPS, driveTemp, steerTemp, driveAmps, steerAmps);

    driveMotor.optimizeBusUtilization();
    steerMotor.optimizeBusUtilization();
  }

  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        steerPos, drivePos, driveVelRPS, driveTemp, steerTemp, driveAmps, steerAmps);

    inputs.driveVelocityMetersPerSec =
        Conversions.RPStoMPS(
            driveVelRPS.getValueAsDouble(),
            swerveConstants.moduleConstants.wheelCircumferenceMeters,
            swerveConstants.moduleConstants.driveGearRatio);
    inputs.driveAppliedVolts = driveVoltageRequest.Output;
    inputs.driveCurrentAmps = driveAmps.getValueAsDouble();
    inputs.driveTempCelcius = driveTemp.getValueAsDouble();
    inputs.driveDistanceMeters =
        Conversions.RotationsToMeters(
            drivePos.getValueAsDouble(),
            swerveConstants.moduleConstants.wheelCircumferenceMeters,
            swerveConstants.moduleConstants.driveGearRatio);
    inputs.driveOutputPercent = driveMotor.get();
    inputs.rawDriveRPS = driveVelRPS.getValueAsDouble();

    inputs.moduleAngleRads =
        Units.degreesToRadians(
            Conversions.RotationsToDegrees(
                steerPos.getValueAsDouble(), swerveConstants.moduleConstants.steerGearRatio));
    inputs.moduleAngleDegs =
        Conversions.RotationsToDegrees(
            steerPos.getValueAsDouble(), swerveConstants.moduleConstants.steerGearRatio);
    inputs.rawAbsolutePositionRotations = absolutePositionRotations.getValueAsDouble();
    inputs.absolutePositionRadians = absolutePositionRotations.getValueAsDouble() * 2 * Math.PI;
    inputs.absolutePositionDegrees = absolutePositionRotations.getValueAsDouble() * 360;
    inputs.turnAppliedVolts = steerVoltageRequest.Output;
    inputs.turnCurrentAmps = steerAmps.getValueAsDouble();
    inputs.turnTempCelcius = steerTemp.getValueAsDouble();
  }

  public void setDesiredState(SwerveModuleState optimizedDesiredStates, boolean isOpenLoop) {
    if (isOpenLoop) {
      double driveVoltage = optimizedDesiredStates.speedMetersPerSecond * 7;
      double angleDeg = optimizedDesiredStates.angle.getDegrees();

      setDriveVoltage(driveVoltage);
      setTurnAngle(angleDeg);
    } else if (!isOpenLoop) {
      double driveVelocity = optimizedDesiredStates.speedMetersPerSecond;
      double angleDeg = optimizedDesiredStates.angle.getDegrees();

      setDriveVelocity(driveVelocity, true);
      setTurnAngle(angleDeg);
    }
  }

  public void setDriveVoltage(double volts) {
    driveMotor.setControl(driveVoltageRequest.withOutput(volts));
  }

  public void steerVoltage(double volts) {
    steerMotor.setControl(steerVoltageRequest.withOutput(volts));
  }

  public void setTurnAngle(double angleDeg) {
    steerMotor.setControl(
        steerRequest.withPosition(
            Conversions.DegreesToRotations(
                angleDeg, swerveConstants.moduleConstants.steerGearRatio)));
  }

  public void resetToAbsolute() {
    double absolutePositionRotations =
        angleEncoder.getAbsolutePosition().getValueAsDouble() - CANcoderOffset;
    double absolutePositionSteerRotations =
        absolutePositionRotations * moduleConstants.steerGearRatio;
    steerMotor.setPosition(absolutePositionSteerRotations);
  }

  public void setDriveVelocity(double velocityMetersPerSecond, boolean auto) {
    velocityVoltageRequest.Velocity =
        Conversions.MPStoRPS(
            velocityMetersPerSecond,
            swerveConstants.moduleConstants.wheelCircumferenceMeters,
            swerveConstants.moduleConstants.driveGearRatio);
    driveMotor.setControl(velocityVoltageRequest);
  }
}
