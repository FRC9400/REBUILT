package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final IntakeIO intakeIO;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;
   }

    @Override
    public void periodic(){
        intakeIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public void requestIntakeVoltage(double volts){
        intakeIO.requestIntakeVoltage(volts);
    }

    public void requestPivotVoltage(double volts){
        intakeIO.requestPivotVoltage(volts);
    }

    public void requestSetpoint(double setpointDeg){
        intakeIO.requestSetpoint(setpointDeg);
    }
    
}
