package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.Logger;
public class Intake {

   private final IntakeIO intakeIO;
   private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
   private double setpointVolts;
   private IntakeStates intakeState = IntakeStates.IDLE;

   public enum IntakeStates {
        IDLE
   }

   public Intake(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;
   }

   public void Loop() {
    intakeIO.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake", this.intakeState);
    switch(intakeState){
        case IDLE:
            intakeIO.requestPivotVoltage(0);
            intakeIO.requestIntakeVoltage(0);
        default:
            break;
    }
   }

   public void requestIdle() {
        setState(IntakeStates.IDLE);
   }

   public void setState(IntakeStates nextState) {
     this.intakeState = nextState;
   }

}
