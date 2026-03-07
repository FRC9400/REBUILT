package frc.robot.Subsystems.Rollers;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.rollersConstants;

public class Rollers extends SubsystemBase {
    private final RollersIO rollersIO;
    private final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();
    private RollersStates rollersState = RollersStates.IDLE;

    public enum RollersStates {
        IDLE,    // Rollers stopped
        PROCESS  // Hopper rollers running to move fuel through
    }

    public Rollers(RollersIO rollersIO) {
        this.rollersIO = rollersIO;
    }

    public void Loop() {
        rollersIO.updateInputs(inputs);
        Logger.processInputs("Rollers", inputs);
        Logger.recordOutput("RollersState", this.rollersState);

        switch (rollersState) {
            case IDLE:
                rollersIO.requestVoltage(0);
                break;

            case PROCESS:
                rollersIO.requestVoltage(rollersConstants.processVoltage);
                break;

            default:
                break;
        }
    }

    public void requestIdle() {
        setState(RollersStates.IDLE);
    }

    public void requestProcess() {
        setState(RollersStates.PROCESS);
    }

    public void setState(RollersStates nextState) {
        this.rollersState = nextState;
    }

    public RollersStates getState() {
        return rollersState;
    }


}
