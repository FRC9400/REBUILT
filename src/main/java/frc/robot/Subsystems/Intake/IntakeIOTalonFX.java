package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.commons.Conversions;
import frc.robot.Constants.intakeConstants;
import frc.robot.Subsystems.Intake.IntakeIO.IntakeIOInputs;
import frc.robot.Constants.canIDConstants;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX pivot = new TalonFX(canIDConstants.pivotMotor, "rio");
    private final TalonFX intake = new TalonFX(canIDConstants.intakeMotor, "rio");
    private final TalonFX intake2 = new TalonFX(19, "rio");

    private final TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
    private final TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();
    
    MotionMagicVoltage pivotMotionMagicRequest;
    VoltageOut pivotVoltageRequest;
    VoltageOut intakeVoltageRequest;

    double pivotSetpoint;
    double intakeSetpointVolts;

    private final StatusSignal<Current> pivotCurrent = pivot.getStatorCurrent();
    private final StatusSignal<Temperature> pivotTemp = pivot.getDeviceTemp();
    private final StatusSignal<AngularVelocity> pivotRPS = pivot.getRotorVelocity();
    private final StatusSignal<Angle> pivotPos = pivot.getRotorPosition();
    private final StatusSignal<Voltage> pivotVoltage = pivot.getMotorVoltage();

    private final StatusSignal<Current> intakeCurrent = intake.getStatorCurrent();
    private final StatusSignal<Temperature> intakeTemp = intake.getDeviceTemp();
    private final StatusSignal<Voltage> intakeVoltage = intake.getMotorVoltage();
    private final StatusSignal<AngularVelocity> intakeRPS = intake.getRotorVelocity();
    
    private final StatusSignal<Current> intake2Current = intake2.getStatorCurrent();
    private final StatusSignal<Temperature> intake2Temp = intake2.getDeviceTemp();
    private final StatusSignal<Voltage> intake2Voltage = intake2.getMotorVoltage();
    private final StatusSignal<AngularVelocity> intake2RPS = intake2.getRotorVelocity();


    public IntakeIOTalonFX() {
        var pivotMotorOutputConfigs = pivotConfigs.MotorOutput;
        pivotMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        pivotMotorOutputConfigs.Inverted = intakeConstants.pivotInvert;

        var pivotCurrentLimitConfigs = pivotConfigs.CurrentLimits;
        pivotCurrentLimitConfigs.StatorCurrentLimit = intakeConstants.pivotStatorCurrentLimit;
        pivotCurrentLimitConfigs.StatorCurrentLimitEnable = true;
        pivotCurrentLimitConfigs.SupplyCurrentLimit = intakeConstants.pivotCurrentLimit;
        pivotCurrentLimitConfigs.SupplyCurrentLimitEnable = true;

        var slot0Configs = pivotConfigs.Slot0;
        slot0Configs.kP = 1.5;
        slot0Configs.kI = 0.0;
        slot0Configs.kD = 0.02;
        slot0Configs.kS = 0.408;
        slot0Configs.kV = 0.0094435;
        slot0Configs.kA = 0.01;
        slot0Configs.kG = 1;
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        var motionMagicConfigs = pivotConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 60.0;
        motionMagicConfigs.MotionMagicAcceleration = 120.0;
        motionMagicConfigs.MotionMagicJerk = 10000.0;

        pivotMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
        pivotVoltageRequest = new VoltageOut(0).withEnableFOC(true);
        intakeVoltageRequest = new VoltageOut(0);

        var feedbackConfigs = pivotConfigs.Feedback;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        var intakeMotorOutputConfigs = intakeConfigs.MotorOutput;
        intakeMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        intakeMotorOutputConfigs.Inverted = intakeConstants.intakeInvert;

        var intakeCurrentLimitConfigs = intakeConfigs.CurrentLimits;
        intakeCurrentLimitConfigs.StatorCurrentLimit = intakeConstants.intakeCurrentLimit;
        intakeCurrentLimitConfigs.StatorCurrentLimitEnable = true;

        pivot.getConfigurator().apply(pivotConfigs);
        intake.getConfigurator().apply(intakeConfigs);
       // intake2.getConfigurator().apply(intakeConfigs); // was commented out, now applied so intake2 gets limits too
        intake2.setControl(new Follower(intake.getDeviceID(), MotorAlignmentValue.Opposed));

        pivot.setPosition(0);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            pivotCurrent,
            pivotPos,
            pivotRPS,
            pivotTemp,
            pivotVoltage,
            intakeTemp,
            intakeCurrent,
            intakeRPS,
            intakeVoltage,
            intake2Temp,
            intake2Current,
            intake2RPS,
            intake2Voltage
        );
        intake.optimizeBusUtilization();
        intake2.optimizeBusUtilization();
        pivot.optimizeBusUtilization();
    }

    public void updateInputs(IntakeIOInputs intakeInputs) {
        BaseStatusSignal.refreshAll(
            pivotCurrent,
            pivotPos,
            pivotRPS,
            pivotTemp,
            pivotVoltage,
            intakeTemp,
            intakeCurrent,
            intakeRPS,
            intakeVoltage,
            intake2Temp,
            intake2Current,
            intake2RPS,
            intake2Voltage
        );
        intakeInputs.pivotAppliedVolts = pivotVoltageRequest.Output;
        intakeInputs.pivotCurrent = pivotCurrent.getValueAsDouble();
        intakeInputs.pivotPosDeg = Conversions.RotationsToDegrees(pivotPos.getValueAsDouble(), intakeConstants.gearRatio);
        intakeInputs.pivotPosRot = pivotPos.getValueAsDouble();
        intakeInputs.pivotSetpointDeg = pivotSetpoint;
        intakeInputs.pivotSetpointRot = Conversions.DegreesToRotations(pivotSetpoint, intakeConstants.gearRatio);
        intakeInputs.pivotTemperature = pivotTemp.getValueAsDouble();
        intakeInputs.pivotRPS = pivotRPS.getValueAsDouble();
        intakeInputs.pivotVoltage = pivotVoltage.getValueAsDouble();

        intakeInputs.intakeTemperature = intakeTemp.getValueAsDouble();
        intakeInputs.intakeAppliedVolts = intakeVoltageRequest.Output;
        intakeInputs.intakeCurrent = intakeCurrent.getValueAsDouble();
        intakeInputs.intakeRPS = intakeRPS.getValueAsDouble();
        intakeInputs.intakeSetpointVolts = this.intakeSetpointVolts;
        intakeInputs.intakeVoltage = intakeVoltage.getValueAsDouble();

        intakeInputs.intake2Current = intake2Current.getValueAsDouble();
        intakeInputs.intake2RPS = intake2RPS.getValueAsDouble();
        intakeInputs.intake2Temperature = intake2Temp.getValueAsDouble();
        intakeInputs.intake2Voltage = intake2Voltage.getValueAsDouble();

    }

    public void requestPivotVoltage(double voltage) {
        pivot.setControl(pivotVoltageRequest.withOutput(voltage));
    }

    public void requestSetpoint(double angleDegrees) {
        this.pivotSetpoint = angleDegrees;
        pivot.setControl(pivotMotionMagicRequest.withPosition(Conversions.DegreesToRotations(angleDegrees, intakeConstants.gearRatio)));
    }

    public void requestIntakeVoltage(double voltage) {
        this.intakeSetpointVolts = voltage;
        intake.setControl(intakeVoltageRequest.withOutput(intakeSetpointVolts));
    }

    public void zeroPosition() {
        pivot.setPosition(0);
    }
}