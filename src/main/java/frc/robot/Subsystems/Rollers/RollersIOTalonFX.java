package frc.robot.Subsystems.Rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.rollersConstants;

public class RollersIOTalonFX implements RollersIO{
    // Motor + Configs
    private TalonFX roller = new TalonFX(1, "rio");

    private TalonFXConfiguration rollerConfigs = new TalonFXConfiguration();

    // Control Reqs
    private VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

    // Setpoints
    private double setpointVolts = 0;

    // Status Signals
    private StatusSignal<Current> rollCurrent = roller.getStatorCurrent();
    private StatusSignal<Temperature> rollTemp = roller.getDeviceTemp();
    private StatusSignal<AngularVelocity> rollRPS = roller.getRotorVelocity();
    private StatusSignal<Voltage> rollVoltage = roller.getMotorVoltage();

    public RollersIOTalonFX(){
        // Configs: Current Limit, Neutral Mode, Invert
        rollerConfigs.CurrentLimits.StatorCurrentLimit = rollersConstants.rollersCurrentLimit;
        rollerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        rollerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerConfigs.MotorOutput.Inverted = rollersConstants.rollersInvert;

        // PIDF Vals
        rollerConfigs.Slot0.kP = 0;
        rollerConfigs.Slot0.kI = 0;
        rollerConfigs.Slot0.kD = 0;
        rollerConfigs.Slot0.kS = 0;
        rollerConfigs.Slot0.kV = 0;
        rollerConfigs.Slot0.kA = 0;
        rollerConfigs.Slot0.kG = 0;
        rollerConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        // Apply Configs
        roller.getConfigurator().apply(rollerConfigs);

        // Freq Updates
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            rollCurrent,
            rollTemp,
            rollRPS,
            rollVoltage,

        // Bus Util
        roller.optimizeBusUtilization();
    }

    public void updateInputs(RollersIOInputs inputs){
        // Refresh Static Signals
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            rollCurrent,
            rollTemp,
            rollRPS,
            rollVoltage);

        // Refresh Inputs
        inputs.appliedVolts = voltageRequest.Output;
        inputs.setpointVolts = setpointVolts;

        inputs.rollerCurrent = rollCurrent.getValueAsDouble();
        inputs.rollerTemp = rollTemp.getValueAsDouble();
        inputs.rollerRPS = rollRPS.getValueAsDouble();
    }

    // Voltage Req
    public void requestVoltage(double volts){
        setpointVolts = volts;
        roller.setControl(voltageRequest.withOutput(volts));
    }
}
