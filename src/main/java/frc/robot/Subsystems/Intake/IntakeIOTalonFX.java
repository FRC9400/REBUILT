package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.intakeConstants;

public class IntakeIOTalonFX implements IntakeIO {
     /* Motor Objects */
    private final TalonFX motor;
    private final TalonFXConfiguration motorConfigs;

    /* Status Signals */
    private final StatusSignal<Current> current;
    private final StatusSignal<Temperature> temp;
    private final StatusSignal<AngularVelocity> angularVelocity;
    private final StatusSignal<Voltage> voltage;

    /* Control Requests */
    private VoltageOut voltageOutRequest;

    /* Doubles */
    private double setpointVolts; 
    
    public IntakeIOTalonFX(){
        /* Motor Objects */
        motor = new TalonFX(canIDConstants.intakeMotor, canIDConstants.canivore);
        motorConfigs = new TalonFXConfiguration();

        /* Status Signals */
        current = motor.getStatorCurrent();
        temp = motor.getDeviceTemp();
        angularVelocity = motor.getRotorVelocity();
        voltage = motor.getMotorVoltage();

        /* Control Requests */
        voltageOutRequest = new VoltageOut(0).withEnableFOC(true);

        /* Doubles */
        setpointVolts = 0;

        /* Current Limit Configuration */
        motorConfigs.CurrentLimits.StatorCurrentLimit = intakeConstants.intakeCurrentLimit;
        motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        /* Motor Output Configuration */
        motorConfigs.MotorOutput.Inverted = intakeConstants.intakeInvert;
        motor.getConfigurator().apply(motorConfigs);

        /* Set Frequency */
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            current,
            temp,
            voltage,
            angularVelocity
        );

        /* Optimize Bus Utilization */
        motor.optimizeBusUtilization();
    }

     public void updateInputs(IntakeInputs inputs){
        /* Refresh Status Signals */
        BaseStatusSignal.refreshAll(
            current,
            temp,
            voltage,
            angularVelocity
        );

        /* Refresh Inputs */
        inputs.appliedVolts = voltageOutRequest.Output;
        inputs.setpointVolts = setpointVolts;
        inputs.voltage = voltage.getValueAsDouble();
        inputs.currentAmps = current.getValueAsDouble();
        inputs.velocityRPS = angularVelocity.getValueAsDouble();
        inputs.tempFahrenheit = temp.getValueAsDouble();
    }

    public void requestVoltage(double volts){
        this.setpointVolts = volts;
        motor.setControl(voltageOutRequest.withOutput(volts));
    }
}
