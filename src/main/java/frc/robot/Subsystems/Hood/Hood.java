package frc.robot.Subsystems.Hood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase{
    private final HoodIO hoodIO;
    private HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
    private double setpointDeg = 0;
    private double setpointVolts = 0;

    public Hood(HoodIO hoodIO){
        this.hoodIO = hoodIO;
    }

    @Override
    public void periodic(){
        hoodIO.updateInputs(hoodInputs);
        Logger.processInputs("Hood", hoodInputs);
    }

    public void requestMotionMagic(double setpointDeg){
        hoodIO.requestMotionMagic(setpointDeg);
    }

    public void requestVoltage(double setpointVolts){
        hoodIO.requestVoltage(setpointVolts);
    }

}