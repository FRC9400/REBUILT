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
    private TalonFX leftRoller = new TalonFX(1, "rio");
    private TalonFX rightRoller = new TalonFX(1, "rio");

    private TalonFXConfiguration rollerConfigs = new TalonFXConfiguration();

    // Control Reqs
    private VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

    // Setpoints
    private double setpointVolts = 0;

    // Status Signals
    private StatusSignal<Current> leftRollCurrent = leftRoller.getStatorCurrent();
    private StatusSignal<Current> rightRollCurrent = rightRoller.getStatorCurrent();
    private StatusSignal<Temperature> leftRollTemp = leftRoller.getDeviceTemp();
    private StatusSignal<Temperature> rightRollTemp = rightRoller.getDeviceTemp();
    private StatusSignal<AngularVelocity> leftRollRPS = leftRoller.getRotorVelocity();
    private StatusSignal<AngularVelocity> rightRollRPS = rightRoller.getRotorVelocity();
    private StatusSignal<Voltage> leftRollVoltage = leftRoller.getMotorVoltage();
    private StatusSignal<Voltage> rightRollVoltage = rightRoller.getMotorVoltage();

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
        leftRoller.getConfigurator().apply(rollerConfigs);
        rightRoller.getConfigurator().apply(rollerConfigs);

        // Freq Updates
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            leftRollCurrent,
            rightRollCurrent,
            leftRollTemp,
            rightRollTemp,
            leftRollRPS,
            rightRollRPS,
            leftRollVoltage,
            rightRollVoltage);

        // Bus Util
        leftRoller.optimizeBusUtilization();
        rightRoller.optimizeBusUtilization();
    }

    public void updateInputs(RollersIOInputs inputs){
        // Refresh Static Signals
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            leftRollCurrent,
            rightRollCurrent,
            leftRollTemp,
            rightRollTemp,
            leftRollRPS,
            rightRollRPS,
            leftRollVoltage,
            rightRollVoltage);

        // Refresh Inputs
        inputs.appliedVolts = voltageRequest.Output;
        inputs.setpointVolts = setpointVolts;

        inputs.leftRollerCurrent = leftRollCurrent.getValueAsDouble();
        inputs.leftRollerTemp = leftRollTemp.getValueAsDouble();
        inputs.leftRollerRPS = leftRollRPS.getValueAsDouble();

        inputs.rightRollerCurrent = rightRollCurrent.getValueAsDouble();
        inputs.rightRollerTemp = rightRollTemp.getValueAsDouble();
        inputs.rightRollerRPS = rightRollRPS.getValueAsDouble();
    }

    // Voltage Req
    public void requestVoltage(double volts){
        setpointVolts = volts;
        leftRoller.setControl(voltageRequest.withOutput(volts));
        rightRoller.setControl(voltageRequest.withOutput(volts));
    }
}
