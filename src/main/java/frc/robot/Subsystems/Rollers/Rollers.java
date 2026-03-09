package frc.robot.Subsystems.Rollers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers {
    private final RollersIO rollersIO;
    private final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();
    private RollersState rollersState = RollersState.IDLE;
    private double voltageSetpoint = 0;

    public Rollers(RollersIO rollersIO){
        this.rollersIO = rollersIO;
    }

    public enum RollersState{
        IDLE,
        VOLTAGE
    }

    public void Loop(){
        rollersIO.updateInputs(inputs);
        Logger.processInputs("Rollers: ", inputs);
        Logger.recordOutput("Rollers", this.rollersState);
        switch(rollersState){
            case IDLE:
                rollersIO.requestVoltage(0);
                break;
            case VOLTAGE:
                rollersIO.requestVoltage(voltageSetpoint);
                break;
            default:
                break;
        }
    }

    public void requestIdle(){
        setState(RollersState.IDLE);
    }

    public void requestVoltage(double volts){
        voltageSetpoint = volts;
        setState(RollersState.VOLTAGE);
    }

    public void setState(RollersState nextState){
        this.rollersState = nextState;
    }
}