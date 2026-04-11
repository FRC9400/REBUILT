package frc.robot.Subsystems.Rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.rollersConstants;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

public class RollersIOTalonFX implements RollersIO{
    // Motor + Configs
    private TalonFX roller = new TalonFX(canIDConstants.rollersMotor, "rio");
    private TalonFX roller2 = new TalonFX(21, "rio");

    private TalonFXConfiguration rollerConfigs = new TalonFXConfiguration();

    // Control Reqs
    private VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
    private MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0).withEnableFOC(true);

    // Setpoints
    private double setpointVolts = 0;
    private double setpointRPS = 0;

    // Status Signals
    private StatusSignal<Current> rollCurrent = roller.getStatorCurrent();
    private StatusSignal<Temperature> rollTemp = roller.getDeviceTemp();
    private StatusSignal<AngularVelocity> rollRPS = roller.getRotorVelocity();
    private StatusSignal<Voltage> rollVoltage = roller.getMotorVoltage();

    private StatusSignal<Current> roll2Current = roller2.getStatorCurrent();
    private StatusSignal<Temperature> roll2Temp = roller2.getDeviceTemp();
    private StatusSignal<AngularVelocity> roll2RPS = roller2.getRotorVelocity();
    private StatusSignal<Voltage> roll2Voltage = roller2.getMotorVoltage();

    public RollersIOTalonFX(){
        // Configs: Current Limit, Neutral Mode, Invert
        rollerConfigs.CurrentLimits.StatorCurrentLimit = rollersConstants.rollersCurrentLimit;
        rollerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        rollerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerConfigs.MotorOutput.Inverted = rollersConstants.rollersInvert;

        rollerConfigs.Slot0.kP = 2;
        rollerConfigs.Slot0.kI = 0;
        rollerConfigs.Slot0.kD = 0;
        rollerConfigs.Slot0.kA = 0;
        rollerConfigs.Slot0.kS = 0;
        rollerConfigs.Slot0.kV = 0;
        rollerConfigs.Slot0.kG = 0;

        var motionMagicConfigs = rollerConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 120.0;
        motionMagicConfigs.MotionMagicJerk = 10000.0;

        // Apply Configs
        roller.getConfigurator().apply(rollerConfigs);

        roller2.setControl(new Follower(roller.getDeviceID(), MotorAlignmentValue.Opposed));

        // Freq Updates
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            rollCurrent,
            rollTemp,
            rollRPS,
            rollVoltage,
            roll2Current,
            roll2Temp,
            roll2RPS,
            roll2Voltage);

        // Bus Util
        roller.optimizeBusUtilization();
        roller2.optimizeBusUtilization();
    }

    public void updateInputs(RollersIOInputs inputs){
        // Refresh Static Signals
        BaseStatusSignal.refreshAll(
            rollCurrent,
            rollTemp,
            rollRPS,
            rollVoltage,
            roll2Current,
            roll2Temp,
            roll2RPS,
            roll2Voltage);

        // Refresh Inputs
        inputs.appliedVolts = voltageRequest.Output;
        inputs.setpointVolts = setpointVolts;
        inputs.setpointRPS = setpointRPS;

        inputs.rollerCurrent = rollCurrent.getValueAsDouble();
        inputs.rollerTemp = rollTemp.getValueAsDouble();
        inputs.rollerVoltage = rollVoltage.getValueAsDouble();
        inputs.rollerRPS = rollRPS.getValueAsDouble();

        inputs.roller2Current = roll2Current.getValueAsDouble();
        inputs.roller2Temp = roll2Temp.getValueAsDouble();
        inputs.roller2Voltage = roll2Voltage.getValueAsDouble();
        inputs.roller2RPS = roll2RPS.getValueAsDouble();
    }

    // Voltage Req
    public void requestVoltage(double volts){
        setpointVolts = volts;
        roller.setControl(voltageRequest.withOutput(volts));
    }

    //MM Req
    public void requestVelocity(double RPS){
        setpointRPS = RPS;
        roller.setControl(velocityRequest.withVelocity(RPS));
    }
}