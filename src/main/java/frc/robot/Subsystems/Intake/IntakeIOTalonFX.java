package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

import frc.commons.Conversions;
import frc.robot.Constants.intakeConstants;
import frc.robot.Subsystems.Intake.IntakeIO.IntakeIOInputs;
import frc.robot.Constants.canIDConstants;

public class IntakeIOTalonFX {
    private final TalonFX pivot = new TalonFX(canIDConstants.pivotMotor, "carnivore");
    private final TalonFX intake = new TalonFX(canIDConstants.intakeMotor, "carnivore");

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

    private final StatusSignal<Current> intakeCurrent = intake.getStatorCurrent();
    private final StatusSignal<Temperature> intakeTemp = pivot.getDeviceTemp();
    private final StatusSignal<AngularVelocity> intakeRPS = pivot.getRotorVelocity();

    public IntakeIOTalonFX() {
        var pivotMotorOutputConfigs = pivotConfigs.MotorOutput;
        pivotMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        pivotMotorOutputConfigs.Inverted = intakeConstants.pivotInvert;

        var pivotCurrentLimitConfigs = pivotConfigs.CurrentLimits;
        pivotCurrentLimitConfigs.StatorCurrentLimit =intakeConstants.pivotCurrentLimit;
        pivotCurrentLimitConfigs.StatorCurrentLimitEnable = true;

        var slot0Configs = pivotConfigs.Slot0;
        //Sys IDs set to 0 for now
        slot0Configs.kP = 0.0;
        slot0Configs.kI = 0.0;
        slot0Configs.kD = 0.0;
        slot0Configs.kS = 0.0;
        slot0Configs.kV = 0.0;
        slot0Configs.kA = 0.0;
        slot0Configs.kG = 0.0;
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        var motionMagicConfigs = pivotConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0.0;
        motionMagicConfigs.MotionMagicAcceleration = 0.0;
        motionMagicConfigs.MotionMagicJerk = 0.0;

        pivotMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
        pivotVoltageRequest = new VoltageOut(0).withEnableFOC(true);
        intakeVoltageRequest = new VoltageOut(0).withEnableFOC(true);


        var feedbackConfigs = pivotConfigs.Feedback;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        pivot.setPosition(0);

        var intakeMotorOutputConfigs = intakeConfigs.MotorOutput;
        intakeMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        intakeMotorOutputConfigs.Inverted = intakeConstants.intakeInvert;

        var intakeCurrentLimitConfigs = pivotConfigs.CurrentLimits;
        intakeCurrentLimitConfigs.StatorCurrentLimit =intakeConstants.intakeCurrentLimit;
        intakeCurrentLimitConfigs.StatorCurrentLimitEnable = true;

        pivot.getConfigurator().apply(pivotConfigs);
        intake.getConfigurator().apply(intakeConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll (
            50,
            pivotCurrent,
            pivotPos,
            pivotRPS,
            pivotTemp,
            intakeTemp,
            intakeCurrent,
            intakeRPS
        );
        intake.optimizeBusUtilization();
        
    }

    public void updateInputs(IntakeIOInputs intakeInputs) {
        BaseStatusSignal.refreshAll (
            pivotCurrent,
            pivotPos,
            pivotRPS,
            pivotTemp,
            intakeTemp,
            intakeCurrent,
            intakeRPS
        );
        intakeInputs.pivotAppliedVolts = pivotVoltageRequest.Output; 
        intakeInputs.pivotCurrent = pivotCurrent.getValueAsDouble();
        intakeInputs.pivotPosDeg = Conversions.RotationsToDegrees(pivotPos.getValueAsDouble(), intakeConstants.gearRatio);
        intakeInputs.pivotPosRot = pivotPos.getValueAsDouble();
        intakeInputs.pivotSetpointDeg = pivotSetpoint;
        intakeInputs.pivotSetpointRot = Conversions.DegreesToRotations(pivotSetpoint, intakeConstants.gearRatio);
        intakeInputs.pivotTemperature = pivotTemp.getValueAsDouble();
        intakeInputs.pivotRPS = pivotRPS.getValueAsDouble();

        intakeInputs.intakeTemperature = intakeTemp.getValueAsDouble();
        intakeInputs.intakeAppliedVolts = intakeVoltageRequest.Output;
        intakeInputs.intakeCurrent = intakeCurrent.getValueAsDouble();
        intakeInputs.intakeRPS = intakeRPS.getValueAsDouble();
        intakeInputs.intakeSetpointVolts = this.intakeSetpointVolts;

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