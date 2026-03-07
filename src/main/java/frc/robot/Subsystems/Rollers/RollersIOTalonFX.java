package frc.robot.Subsystems.Rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.rollersConstants;

public class RollersIOTalonFX implements RollersIO {
    // Motor + Configs
    private TalonFX roller = new TalonFX(canIDConstants.rollersMotor, "rio");
    private TalonFXConfiguration rollerConfigs = new TalonFXConfiguration();

    // Control Requests
    private VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

    // Setpoints
    private double setpointVolts = 0;

    // Status Signals
    private StatusSignal<Current> rollCurrent = roller.getStatorCurrent();
    private StatusSignal<Temperature> rollTemp = roller.getDeviceTemp();
    private StatusSignal<AngularVelocity> rollRPS = roller.getRotorVelocity();
    private StatusSignal<Voltage> rollVoltage = roller.getMotorVoltage();

    public RollersIOTalonFX() {
        rollerConfigs.CurrentLimits.StatorCurrentLimit = rollersConstants.rollersCurrentLimit;
        rollerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerConfigs.MotorOutput.Inverted = rollersConstants.rollersInvert;

        roller.getConfigurator().apply(rollerConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            rollCurrent,
            rollTemp,
            rollRPS,
            rollVoltage);

        roller.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(RollersIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            rollCurrent,
            rollTemp,
            rollRPS,
            rollVoltage);

        inputs.appliedVolts = rollVoltage.getValueAsDouble();
        inputs.setpointVolts = setpointVolts;
        inputs.rollerCurrent = rollCurrent.getValueAsDouble();
        inputs.rollerTemp = rollTemp.getValueAsDouble();
        inputs.rollerRPS = rollRPS.getValueAsDouble();
    }

    @Override
    public void requestVoltage(double volts) {
        setpointVolts = volts;
        roller.setControl(voltageRequest.withOutput(volts));
    }
}
