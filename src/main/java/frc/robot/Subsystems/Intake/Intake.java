package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private final IntakeIO intakeIO;
    private IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();
    private double voltageSetpoint = 0;

    public Intake(IntakeIO intakeIO){
        this.intakeIO = intakeIO;
    }

    @Override
    public void periodic(){
        intakeIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public void requestVoltage(double volts){
        voltageSetpoint = volts;
        intakeIO.requestVoltage(volts);
    }
    
}
