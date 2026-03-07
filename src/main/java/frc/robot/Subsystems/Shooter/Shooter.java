package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.shooterConstants;

public class Shooter extends SubsystemBase {
    private final ShooterIO shooterIO;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private ShooterStates shooterState = ShooterStates.IDLE;

    public enum ShooterStates {
        IDLE,   // Stop flywheels
        PREP,   // Flywheels off — hood/rollers handled externally
        SHOOT   // Flywheels spin — auto-transitions to ZERO when current drops
    }

    public Shooter(ShooterIO shooterIO) {
        this.shooterIO = shooterIO;
    }

    public void Loop() {
        shooterIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("ShooterState", this.shooterState);

        switch (shooterState) {
            case IDLE:
                shooterIO.zeroVelocity();
                break;

            case PREP:
                // Flywheels idle — positioning handled by Hood/Rollers externally
                shooterIO.zeroVelocity();
                break;

            case SHOOT:
                shooterIO.setVelocity(shooterConstants.shootVelocityMPS, shooterConstants.shootRatio);
                // When both stator currents drop below threshold, algae has left the shooter
                if (inputs.currentAmps[0] < shooterConstants.shootCurrentThresholdAmps
                        && inputs.currentAmps[1] < shooterConstants.shootCurrentThresholdAmps) {
                    setState(ShooterStates.IDLE);
                }
                break;

            default:
                break;
        }
    }

    public void requestIdle() {
        setState(ShooterStates.IDLE);
    }

    public void requestPrep() {
        setState(ShooterStates.PREP);
    }

    public void requestShoot() {
        setState(ShooterStates.SHOOT);
    }

    public void setState(ShooterStates nextState) {
        this.shooterState = nextState;
    }

    public ShooterStates getState() {
        return shooterState;
    }

}
