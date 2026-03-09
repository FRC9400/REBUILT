package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.intakeConstants;

public class Intake {
    private final IntakeIO intakeIO;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private IntakeStates intakeState = IntakeStates.IDLE;
    private double voltageSetpoint = 0;
    private double setpointDeg = 0;

    public Intake(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;
    }

    public enum IntakeStates{
        IDLE,
        LOWERED,
        RAISED,
        SETPOINT,
        INTAKE
    }

    public void Loop(){
        intakeIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        Logger.recordOutput("Intake", this.intakeState);
        switch(intakeState){
            case IDLE:
                intakeIO.requestIntakeVoltage(0);
                intakeIO.requestPivotVoltage(0);
                break;
            case LOWERED:
                intakeIO.requestIntakeVoltage(0);
                intakeIO.requestSetpoint(intakeConstants.maxDeg);
                break;
            case RAISED:
                intakeIO.requestIntakeVoltage(0);
                intakeIO.requestSetpoint(intakeConstants.minDeg);
                break;
            case INTAKE:
                intakeIO.requestIntakeVoltage(voltageSetpoint);
                intakeIO.requestSetpoint(intakeConstants.maxDeg);
                break;
            case SETPOINT:
                intakeIO.requestIntakeVoltage(0);
                intakeIO.requestSetpoint(setpointDeg);
                break;
            default:
                break;
        }

    }

    public void requestIdle(){
        setState(IntakeStates.IDLE);
    }

    public void requestLowered(){
        setState(IntakeStates.LOWERED);
    }

    public void requestRaised(){
        setState(IntakeStates.RAISED);
    }
    
    public void requestIntake(double volts){
        voltageSetpoint = volts;
        setState(IntakeStates.INTAKE);
    }

    public void requestSetpoint(double deg){
        setpointDeg = deg;
        setState(IntakeStates.SETPOINT);
    }

    public void setState(IntakeStates nextState){
        this.intakeState = nextState;
    }

}
