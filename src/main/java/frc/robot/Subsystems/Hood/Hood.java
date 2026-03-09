package frc.robot.Subsystems.Hood;

import org.littletonrobotics.junction.Logger;

public class Hood {
    private final HoodIO hoodIO;
    private HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
    private HoodStates hoodState = HoodStates.IDLE;
    private double setpointDegrees = 0;

    public Hood(HoodIO hoodIO){
        this.hoodIO = hoodIO;
    
    }

    public enum HoodStates{
        IDLE,
        ZERO,
        SETPOINT;
    }

    public void Loop(){
        hoodIO.updateInputs(hoodInputs);
        Logger.processInputs("Hood", hoodInputs);
        Logger.recordOutput("Hood", this.hoodState);
        switch(hoodState){
            case IDLE:
                hoodIO.requestVoltage(0);
                break;
            case ZERO:
                hoodIO.requestMotionMagic(0);
                break;
            case SETPOINT:
                hoodIO.requestMotionMagic(setpointDegrees);
            default:
                break;
        }
    }

    public boolean atSetpoint(){
        return Math.abs(hoodInputs.hoodPosDeg - setpointDegrees) < 0.2;
    }

    public void requestIdle(){
        setState(HoodStates.IDLE);
    }
    
    public void requestZero(){
        setState(HoodStates.ZERO);
    }

    public void requestSetpoint(double setpointDeg){
        setpointDegrees = setpointDeg;
        setState(HoodStates.SETPOINT);
    }

    public void setState(HoodStates nextState){
        this.hoodState = nextState;
    }

}