package frc.robot.Subsystems.Hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.databind.JsonSerializable.Base;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.commons.Conversions;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.hoodConstants;

public class HoodIOTalonFX implements HoodIO{
    // Motor + Configs
    private final TalonFX hoodMotor = new TalonFX(0, "rio"); // need update id and canbus
    private final TalonFXConfiguration hoodConfigs = new TalonFXConfiguration();

    // Control Reqs
    private VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
    private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withEnableFOC(true);

    // Setpoints
    private double setpointVolts = 0;
    private double setpointDeg = 0;

    // Status Signals
    private final StatusSignal<Current> hoodCurrent = hoodMotor.getStatorCurrent();
    private final StatusSignal<Temperature> hoodTemp = hoodMotor.getDeviceTemp();
    private final StatusSignal<AngularVelocity> hoodAngularVelocity = hoodMotor.getRotorVelocity();
    private final StatusSignal<Voltage> hoodVoltage = hoodMotor.getMotorVoltage();
    private final StatusSignal<Angle> hoodPosition = hoodMotor.getRotorPosition();

    public HoodIOTalonFX(){
        // Configs: Current Limit, Neutral Mode, Invert
        hoodConfigs.CurrentLimits.StatorCurrentLimit = hoodConstants.hoodCurrentLimit;
        hoodConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        
        hoodConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodConfigs.MotorOutput.Inverted = hoodConstants.hoodInvert;

        // PIDF Values (Slot 0)
        hoodConfigs.Slot0.kP = 0;
        hoodConfigs.Slot0.kI = 0;
        hoodConfigs.Slot0.kD = 0;
        hoodConfigs.Slot0.kS = 0;
        hoodConfigs.Slot0.kV = 0;
        hoodConfigs.Slot0.kA = 0;
        hoodConfigs.Slot0.kG = 0;
        hoodConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        // Apply Configs
        hoodMotor.getConfigurator().apply(hoodConfigs);

        // Updates
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            hoodCurrent,
            hoodTemp,
            hoodAngularVelocity,
            hoodVoltage,
            hoodPosition);

        // Bus Util
        hoodMotor.optimizeBusUtilization();
    }

    public void updateInputs(HoodIOInputs hoodInputs){
        // Refresh Status Signals
        BaseStatusSignal.refreshAll(
            hoodCurrent,
            hoodTemp,
            hoodAngularVelocity,
            hoodVoltage,
            hoodPosition
        );

        // Refresh Inputs
        hoodInputs.appliedDeg = motionMagicRequest.Position;
        hoodInputs.appliedVolts = voltageRequest.Output;

        hoodInputs.setpointVolts = setpointVolts;
        hoodInputs.setpointDeg = setpointDeg;
        hoodInputs.setpointRot = Conversions.DegreesToRotations(setpointDeg, hoodConstants.gearRatio);
    
        hoodInputs.current = hoodCurrent.getValueAsDouble();
        hoodInputs.temp = hoodTemp.getValueAsDouble();
        hoodInputs.rps = hoodAngularVelocity.getValueAsDouble();
    }

    // Voltage Request
    public void requestVoltage(double volts){
        setpointVolts = volts;
        hoodMotor.setControl(voltageRequest.withOutput(setpointVolts));
    }

    // Motion Magic Request
    public void requestMotionMagic(double degrees){
        setpointDeg = degrees;
        hoodMotor.setControl(motionMagicRequest.withPosition(Conversions.DegreesToRotations(setpointDeg, hoodConstants.gearRatio)));
    }

}
