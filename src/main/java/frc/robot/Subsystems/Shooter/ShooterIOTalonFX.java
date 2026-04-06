package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.commons.Conversions;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.shooterConstants;

public class ShooterIOTalonFX implements ShooterIO{
    private final TalonFX leftMotor = new TalonFX(canIDConstants.leftShooterMotor, "rio");
    private final TalonFX rightMotor = new TalonFX(canIDConstants.rightShooterMotor, "rio");
    private TalonFXConfiguration leftMotorConfigs = new TalonFXConfiguration();

    private VoltageOut shootRequestVoltage = new VoltageOut(0).withEnableFOC(true);
    private VelocityVoltage leftShootRequestVelocity = new VelocityVoltage(0).withEnableFOC(true);
    private final MotionMagicVelocityVoltage leftShootRequestMMVelocity =
            new MotionMagicVelocityVoltage(0).withEnableFOC(true);

    private final StatusSignal<Current> leftShooterCurrent = leftMotor.getStatorCurrent();
    private final StatusSignal<Current> rightShooterCurrent = rightMotor.getStatorCurrent();
    private final StatusSignal<Temperature> leftShooterTemp = leftMotor.getDeviceTemp();
    private final StatusSignal<Temperature> rightShooterTemp = rightMotor.getDeviceTemp();
    private final StatusSignal<AngularVelocity> leftShooterSpeedRPS = leftMotor.getRotorVelocity();
    private final StatusSignal<AngularVelocity> rightShooterSpeedRPS = rightMotor.getRotorVelocity();
    private final StatusSignal<Voltage> leftVoltage = leftMotor.getMotorVoltage();
    private final StatusSignal<Voltage> rightVoltage = rightMotor.getMotorVoltage();
    
    private double leftShooterSetpointMPS;

    public ShooterIOTalonFX() {

        leftMotorConfigs.CurrentLimits.StatorCurrentLimit = 70;
        leftMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        leftMotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // 70A burst, drops to 40A after 1s of continuous limiting TUNE
        leftMotorConfigs.CurrentLimits.SupplyCurrentLimit = 70;
        leftMotorConfigs.CurrentLimits.SupplyCurrentLowerLimit = 40;
        leftMotorConfigs.CurrentLimits.SupplyCurrentLowerTime = 1.0;
        leftMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        leftMotorConfigs.Voltage.PeakForwardVoltage = 8;
        leftMotorConfigs.Voltage.PeakReverseVoltage = -8;
        leftMotorConfigs.Slot0.kP = 0.29;
        leftMotorConfigs.Slot0.kD = 0.01;
        leftMotorConfigs.Slot0.kS = 0.2;
        leftMotorConfigs.Slot0.kV = 0.115;
        leftMotorConfigs.Slot0.kA = 0;

         leftMotorConfigs.MotionMagic.MotionMagicAcceleration =
                Conversions.MPStoRPS(15, //TUNE
                        shooterConstants.wheelCircumferenceMeters, 1);
        leftMotorConfigs.MotionMagic.MotionMagicJerk =
                Conversions.MPStoRPS(30, //TUNE
                        shooterConstants.wheelCircumferenceMeters, 1);

        leftMotor.getConfigurator().apply(leftMotorConfigs);
        
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            leftShooterCurrent,
            rightShooterCurrent,
            leftShooterTemp,
            rightShooterTemp,
            leftShooterSpeedRPS,
            rightShooterSpeedRPS,
            leftVoltage,
            rightVoltage
        );

        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();

        leftShooterSetpointMPS = 0;
    }   

    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            leftShooterCurrent,
            rightShooterCurrent,
            leftShooterTemp,
            rightShooterTemp,
            leftShooterSpeedRPS,
            rightShooterSpeedRPS,
            leftVoltage,
            rightVoltage
        );

        inputs.appliedVolts = shootRequestVoltage.Output;
        inputs.appliedVelocity = leftShootRequestVelocity.Velocity;
        inputs.currentAmps = new double[] { leftShooterCurrent.getValueAsDouble(),
                rightShooterCurrent.getValueAsDouble() };
        inputs.temp = new double[] { leftShooterTemp.getValueAsDouble(),
                rightShooterTemp.getValueAsDouble() };
        inputs.shooterVelMPS = new double[] {Conversions.RPStoMPS(leftShooterSpeedRPS.getValueAsDouble(), shooterConstants.wheelCircumferenceMeters, 1), Conversions.RPStoMPS(rightShooterSpeedRPS.getValueAsDouble(), shooterConstants.wheelCircumferenceMeters, 1)};
        inputs.shooterSetpointMPS = leftShooterSetpointMPS;
        inputs.shooterVelRPS = new double[] {leftShooterSpeedRPS.getValueAsDouble(), rightShooterSpeedRPS.getValueAsDouble()};
        inputs.shooterVoltage = new double[] {leftVoltage.getValueAsDouble(), rightVoltage.getValueAsDouble()};
    }

    public void requestVelocity(double velocity) {
        this.leftShooterSetpointMPS = velocity;
        leftMotor.setControl(leftShootRequestVelocity.withVelocity(Conversions.MPStoRPS(velocity, shooterConstants.wheelCircumferenceMeters, 1)));
    }

    public void zeroVelocity(){
        this.leftShooterSetpointMPS = 0;
        leftMotor.setControl(leftShootRequestVelocity.withVelocity(0));
    }

    public void requestMMVelocity(double velocityMPS) {
        this.leftShooterSetpointMPS = velocityMPS;
        leftMotor.setControl(leftShootRequestMMVelocity.withVelocity(
                Conversions.MPStoRPS(velocityMPS, shooterConstants.wheelCircumferenceMeters, 1)));
    }

    public void requestVoltage(double volts){
        leftMotor.setControl(shootRequestVoltage.withOutput(volts));
    }
}