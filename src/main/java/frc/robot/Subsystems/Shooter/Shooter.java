package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;

public class Shooter {
    private final ShooterIO shooterIO;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private ShooterStates shooterStates = ShooterStates.IDLE;
    private double setpointVelocity;
    private double setpointVolts;

    public Shooter(ShooterIO shooterIO){
        this.shooterIO = shooterIO;
    }

    public enum ShooterStates{
        IDLE,
        ZERO,
        VOLTAGE,
        VELOCITY,
    }

    public void Loop(){
        shooterIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("Shooter", this.shooterStates);
        switch(shooterStates){
            case IDLE:
                shooterIO.requestVoltage(0);
                break;
            case ZERO:
                shooterIO.requestVelocity(0);
                break;
            case VOLTAGE:
                shooterIO.requestVoltage(setpointVolts);
                break;
            case VELOCITY:
                shooterIO.requestVelocity(setpointVelocity);
                break;
            default:
                break;
        }
    }

    public boolean atSetpoint(){
        return Math.abs(inputs.shooterVelMPS[0] - setpointVelocity) < 0.5;
    }

    public void requestIdle(){
        setState(ShooterStates.IDLE);
    }

    public void requestZero(){
        setState(ShooterStates.ZERO);
    }

    public void requestVoltage(double volts){
        setpointVolts = volts;
        setState(ShooterStates.VOLTAGE);
    }

    public void requestVelocity(double velocity){
        setpointVelocity = velocity;
        setState(ShooterStates.VELOCITY);
    }

    public void setState(ShooterStates nextState){
        this.shooterStates = nextState;
    }
}