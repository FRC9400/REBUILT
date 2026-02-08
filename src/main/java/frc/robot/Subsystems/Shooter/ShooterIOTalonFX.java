package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

import frc.commons.Conversions;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.shooterConstants;

public class ShooterIOTalonFX implements ShooterIO{
    private final TalonFX leftMotor = new TalonFX(canIDConstants.leftShooterMotor, "canivore");
    private final TalonFX rightMotor = new TalonFX(canIDConstants.rightShooterMotor, "canivore");
    private TalonFXConfiguration leftMotorConfigs = new TalonFXConfiguration();
    private TalonFXConfiguration rightMotorConfigs = new TalonFXConfiguration();

    private VoltageOut shootRequestVoltage = new VoltageOut(0).withEnableFOC(true);
    private VelocityVoltage leftShootRequestVelocity = new VelocityVoltage(0).withEnableFOC(true);
    private VelocityVoltage rightShootRequestVelocity = new VelocityVoltage(0).withEnableFOC(true);

    private final StatusSignal<Current> leftShooterCurrent = leftMotor.getStatorCurrent();
    private final StatusSignal<Current> rightShooterCurrent = rightMotor.getStatorCurrent();
    private final StatusSignal<Temperature> leftShooterTemp = leftMotor.getDeviceTemp();
    private final StatusSignal<Temperature> rightShooterTemp = rightMotor.getDeviceTemp();
    private final StatusSignal<AngularVelocity> leftShooterSpeedRPS = leftMotor.getRotorVelocity();
    private final StatusSignal<AngularVelocity> rightShooterSpeedRPS = rightMotor.getRotorVelocity();
    
    private double leftShooterSetpointMPS;
    private double rightShooterSetpointMPS;

    public ShooterIOTalonFX() {

        leftMotorConfigs.CurrentLimits.StatorCurrentLimit = shooterConstants.statorCurrentLimit;
        leftMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        leftMotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftMotorConfigs.Slot0.kP = shooterConstants.kP;
        leftMotorConfigs.Slot0.kD = shooterConstants.kD;
        leftMotorConfigs.Slot0.kS = shooterConstants.kS;
        leftMotorConfigs.Slot0.kV = shooterConstants.kV;
        leftMotorConfigs.Slot0.kA = shooterConstants.kA;

        rightMotorConfigs.CurrentLimits.StatorCurrentLimit = shooterConstants.statorCurrentLimit;
        rightMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        rightMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightMotorConfigs.Slot0.kP = shooterConstants.kP;
        rightMotorConfigs.Slot0.kD = shooterConstants.kD;
        rightMotorConfigs.Slot0.kS = shooterConstants.kS;
        rightMotorConfigs.Slot0.kV = shooterConstants.kV;
        rightMotorConfigs.Slot0.kA = shooterConstants.kA;

        leftMotor.getConfigurator().apply(leftMotorConfigs);
        rightMotor.getConfigurator().apply(rightMotorConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            leftShooterCurrent,
            rightShooterCurrent,
            leftShooterTemp,
            rightShooterTemp,
            leftShooterSpeedRPS,
            rightShooterSpeedRPS
        );

        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();

        leftShooterSetpointMPS = 0;
        rightShooterSetpointMPS = 0;
    }   

    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            leftShooterCurrent,
            rightShooterCurrent,
            leftShooterTemp,
            rightShooterTemp,
            leftShooterSpeedRPS,
            rightShooterSpeedRPS
        );

        inputs.appliedVolts = shootRequestVoltage.Output;
        inputs.currentAmps = new double[] { leftShooterCurrent.getValueAsDouble(),
                rightShooterCurrent.getValueAsDouble() };
        inputs.tempF = new double[] { leftShooterTemp.getValueAsDouble(),
                rightShooterTemp.getValueAsDouble() };
        inputs.shooterVelMPS = new double[] {Conversions.RPStoMPS(leftShooterSpeedRPS.getValueAsDouble(), shooterConstants.wheelCircumferenceMeters, 1), Conversions.RPStoMPS(rightShooterSpeedRPS.getValueAsDouble(), shooterConstants.wheelCircumferenceMeters, 1)};
        inputs.shooterSetpointsMPS = new double[] {leftShooterSetpointMPS, rightShooterSetpointMPS};
    }

    public void setVelocity(double velocity, double ratio) {
        this.leftShooterSetpointMPS = velocity;
        this.rightShooterSetpointMPS = velocity * ratio;
        // Control both motors with left as master velocity reference
        leftMotor.setControl(leftShootRequestVelocity.withVelocity(Conversions.MPStoRPS(velocity, shooterConstants.wheelCircumferenceMeters, 1)));
        rightMotor.setControl(rightShootRequestVelocity.withVelocity(Conversions.MPStoRPS(velocity * ratio, shooterConstants.wheelCircumferenceMeters, 1)));
    }

    public void zeroVelocity(){
        this.leftShooterSetpointMPS = 0;
        this.rightShooterSetpointMPS = 0;
        // Stop both motors
        leftMotor.setControl(leftShootRequestVelocity.withVelocity(0));
        rightMotor.setControl(rightShootRequestVelocity.withVelocity(0));
    }
}
