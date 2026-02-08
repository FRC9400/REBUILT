package frc.robot.Subsystems.Rollers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase{
    private final RollersIO rollersIO;
    private final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();
    private double setpointVolts = 0;

    public Rollers(RollersIO rollersIO){
        this.rollersIO = rollersIO;
    }

    @Override
    public void periodic(){
        rollersIO.updateInputs(inputs);
        Logger.processInputs("Rollers: ", inputs);
    }

    public void requestVoltage(double setpointVolts){
        rollersIO.requestVoltage(setpointVolts);
    }
}