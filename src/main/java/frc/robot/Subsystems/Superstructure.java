package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.commons.LaunchCalculator;
import frc.commons.LoggedTunableNumber;
import frc.robot.Subsystems.Hood.Hood;
import frc.robot.Subsystems.Hood.HoodIO;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeIO;
import frc.robot.Subsystems.Rollers.Rollers;
import frc.robot.Subsystems.Rollers.RollersIO;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterIO;

public class Superstructure extends SubsystemBase {
    private Hood s_hood;
    private Intake s_intake;
    private Rollers s_rollers;
    private Shooter s_shooter;

    private double stateStartTime = 0;
    private SuperstructureStates systemState = SuperstructureStates.IDLE;

    private Double lookaheadDistanceOverride = null;
    private boolean preSpinEnabled = false;

    // Intake
    LoggedTunableNumber INTAKEintakeVoltage  = new LoggedTunableNumber("Superstructure/INTAKE Intake Voltage", 7);
    LoggedTunableNumber INTAKEintakeRPS = new LoggedTunableNumber("Superstructure/INTAKE Intake RPS", 50);

    // Spit out
    LoggedTunableNumber SPITOUTintakeVoltage  = new LoggedTunableNumber("Superstructure/SPITOUT Intake Voltage", 7);
    LoggedTunableNumber SPITOUTintakeRPS = new LoggedTunableNumber("Superstructure/INTAKE Intake RPS", 7);

    LoggedTunableNumber SPITOUTrollersVoltage = new LoggedTunableNumber("Superstructure/SPITOUT Rollers Voltage", -6);
    LoggedTunableNumber SPITOUTrollersRPS = new LoggedTunableNumber("Superstructure/SPITOUT Rollers RPS", -50);

    // Unjam
    LoggedTunableNumber UNJAMrollersVoltage = new LoggedTunableNumber("Superstructure/UNJAM Rollers Voltage", -8);
    LoggedTunableNumber UNJAMshooterVoltage = new LoggedTunableNumber("Superstructure/UNJAM Shooter Voltage", -4);

    // Teleop shoot
    LoggedTunableNumber hoodSetpoint        = new LoggedTunableNumber("Superstructure/SPINUP AND SHOOT Hood Setpoint Deg", 25);
    LoggedTunableNumber shooterVelocity     = new LoggedTunableNumber("Superstructure/SPINUP AND SHOOT Shooter Velocity", 18);
    LoggedTunableNumber SHOOTRollersVoltage = new LoggedTunableNumber("Superstructure/SHOOT Rollers Voltage", 6);
    LoggedTunableNumber SHOOTIntakeVoltage  = new LoggedTunableNumber("Superstructure/SHOOT Intake Voltage", 2);
    LoggedTunableNumber SHOOTRollersRPS = new LoggedTunableNumber("Superstructure/SHOOT Rollers RPS", 40);
    LoggedTunableNumber SHOOTIntakeRPS  = new LoggedTunableNumber("Superstructure/SHOOT Intake RPS", 20);

    // Auto shoot — separate so we can lower draw independently
    LoggedTunableNumber autoSHOOTRollersVoltage = new LoggedTunableNumber("Superstructure/AUTO SHOOT Rollers Voltage", 6);
    LoggedTunableNumber autoSHOOTIntakeVoltage  = new LoggedTunableNumber("Superstructure/AUTO SHOOT Intake Voltage", 1.5);
    LoggedTunableNumber autoSHOOTRollersRPS = new LoggedTunableNumber("Superstructure/AUTO SHOOT Rollers Voltage", 40);
    LoggedTunableNumber autoSHOOTIntakeRPS  = new LoggedTunableNumber("Superstructure/AUTO SHOOT Intake Voltage", 10);

    // Shooter velocity offset — subtract from interpolated velocity to fix overshoot
    LoggedTunableNumber shooterVelocityOffset = new LoggedTunableNumber("Superstructure/AUTO Shooter Velocity Offset", 0.0);

    LoggedTunableNumber minVelocity = new LoggedTunableNumber("Superstructure/MIN VELOCITY FOR SPINUP", 12.7);

    private final DoubleSupplier distanceSupplier;
    private final DoubleSupplier distanceToPassTargetSupplier;

    public Superstructure(HoodIO hoodIO, IntakeIO intakeIO, RollersIO rollersIO, ShooterIO shooterIO,
            DoubleSupplier distanceSupplier, DoubleSupplier distanceToPassTargetSupplier) {
        this.s_hood     = new Hood(hoodIO);
        this.s_intake   = new Intake(intakeIO);
        this.s_rollers  = new Rollers(rollersIO);
        this.s_shooter  = new Shooter(shooterIO);
        this.distanceSupplier = distanceSupplier;
        this.distanceToPassTargetSupplier = distanceToPassTargetSupplier;
    }

    public enum SuperstructureStates {
        IDLE,
        BUMP,
        INTAKE,
        SPITOUT,
        UN_JAM,
        SPIN_UP,
        SHOOT_A,
        SHOOT_B,
        AUTO_SPIN_UP,
        AUTO_SHOOT_A,
        AUTO_SHOOT_B,
        AUTO_PASS_SPIN_UP,
        AUTO_PASS_A,
        AUTO_PASS_B,
        PIVOT_SHAKING
    }

    @Override
    public void periodic() {
        s_hood.Loop();
        s_intake.Loop();
        s_rollers.Loop();
        s_shooter.Loop();

        double distance = lookaheadDistanceOverride != null
                ? lookaheadDistanceOverride
                : distanceSupplier.getAsDouble();

        double passingDistance = lookaheadDistanceOverride != null
                ? lookaheadDistanceOverride
                : distanceToPassTargetSupplier.getAsDouble();

        double interpolatedHoodAngle = LaunchCalculator.getHoodAngleDeg(distance);
        double interpolatedShooterVelocity = LaunchCalculator.getShooterVelocity(distance);

        double interpolatedPassingHoodAngle = LaunchCalculator.getHoodAngleDeg(passingDistance);
        double interpolatedPassingShooterVelocity = LaunchCalculator.getShooterVelocity(passingDistance);

        Logger.recordOutput("SuperstructureState", this.systemState);
        Logger.recordOutput("Superstructure/DistanceToHub", distance);
        Logger.recordOutput("Superstructure/UsingLookahead", lookaheadDistanceOverride != null);
        Logger.recordOutput("Superstructure/InterpolatedHoodAngle", interpolatedHoodAngle);
        Logger.recordOutput("Superstructure/InterpolatedShooterVelocity", interpolatedShooterVelocity);
        Logger.recordOutput("Superstructure/InterpolatedPASSINGHoodAngle", interpolatedPassingHoodAngle);
        Logger.recordOutput("Superstructure/InterpolatedPASSINGShooterVelocity", interpolatedPassingShooterVelocity);
        Logger.recordOutput("Superstructure/ShooterVelocityOffset", shooterVelocityOffset.getAsDouble());

        switch (systemState) {
            case IDLE:
                s_hood.requestIdle();
                s_intake.requestIdle();
                s_rollers.requestIdle();
                handleIdleShooter();
                break;

            case BUMP:
                s_hood.requestIdle();
                s_intake.requestRaised();
                s_rollers.requestIdle();
                handleIdleShooter();
                break;

            case INTAKE:
                s_hood.requestIdle();
                s_intake.requestIntake(INTAKEintakeVoltage.getAsDouble());
                s_rollers.requestIdle();
                handleIdleShooter();
                break;

            case SPITOUT:
                s_hood.requestIdle();
                s_intake.requestIntake(-SPITOUTintakeVoltage.getAsDouble());
                s_rollers.requestVoltage(SPITOUTrollersVoltage.getAsDouble());
                handleIdleShooter();
                break;

            case UN_JAM:
                s_hood.requestIdle();
                s_intake.requestLowered();
                s_rollers.requestVoltage(UNJAMrollersVoltage.getAsDouble());
                s_shooter.requestVoltage(UNJAMshooterVoltage.getAsDouble());
                break;

            case SPIN_UP:
                s_hood.requestSetpoint(hoodSetpoint.getAsDouble());
                s_intake.requestLowered();
                s_rollers.requestIdle();
                s_shooter.requestMMVelocity(shooterVelocity.getAsDouble());
                if (s_shooter.atSetpoint()) {
                    setState(SuperstructureStates.SHOOT_A);
                }
                break;

            case SHOOT_A:
                s_hood.requestSetpoint(hoodSetpoint.getAsDouble());
                s_intake.requestSetpoint(SHOOTIntakeVoltage.getAsDouble(), 75);
                s_rollers.requestVoltage(SHOOTRollersVoltage.getAsDouble());
                s_shooter.requestMMVelocity(shooterVelocity.getAsDouble());
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.5) {
                    setState(SuperstructureStates.SHOOT_B);
                }
                break;

            case SHOOT_B:
                s_hood.requestSetpoint(hoodSetpoint.getAsDouble());
                s_intake.requestSetpoint(SHOOTIntakeVoltage.getAsDouble(), 135);
                s_rollers.requestVoltage(SHOOTRollersVoltage.getAsDouble());
                s_shooter.requestMMVelocity(shooterVelocity.getAsDouble());
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.5) {
                    setState(SuperstructureStates.SHOOT_A);
                }
                break;

            // ----------------------------------------------------------------
            // Auto shooting
            // ----------------------------------------------------------------
            case AUTO_SPIN_UP:
                s_intake.requestLowered();
                s_rollers.requestIdle();
                s_hood.requestSetpoint(interpolatedHoodAngle);
                s_shooter.requestMMVelocity(interpolatedShooterVelocity - shooterVelocityOffset.getAsDouble());
                if (s_shooter.atSetpoint()) {
                    setState(SuperstructureStates.AUTO_SHOOT_A);
                }
                break;

            case AUTO_SHOOT_A:
                s_hood.requestSetpoint(interpolatedHoodAngle);
                s_intake.requestSetpoint(autoSHOOTIntakeVoltage.getAsDouble(), 75);
                s_rollers.requestVoltage(autoSHOOTRollersVoltage.getAsDouble());
                s_shooter.requestMMVelocity(interpolatedShooterVelocity - shooterVelocityOffset.getAsDouble());
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.5) {
                    setState(SuperstructureStates.AUTO_SHOOT_B);
                }
                break;

            case AUTO_SHOOT_B:
                s_hood.requestSetpoint(interpolatedHoodAngle);
                s_intake.requestSetpoint(autoSHOOTIntakeVoltage.getAsDouble(), 135);
                s_rollers.requestVoltage(autoSHOOTRollersVoltage.getAsDouble());
                s_shooter.requestMMVelocity(interpolatedShooterVelocity - shooterVelocityOffset.getAsDouble());
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.5) {
                    setState(SuperstructureStates.AUTO_SHOOT_A);
                }
                break;

            // ----------------------------------------------------------------
            // Auto passing
            // ----------------------------------------------------------------
            case AUTO_PASS_SPIN_UP:
                s_intake.requestLowered();
                s_rollers.requestIdle();
                s_hood.requestSetpoint(interpolatedPassingHoodAngle);
                s_shooter.requestVelocity(interpolatedPassingShooterVelocity);
                if (s_shooter.atSetpoint()) {
                    setState(SuperstructureStates.AUTO_PASS_A);
                }
                break;

            case AUTO_PASS_A:
                s_hood.requestSetpoint(interpolatedPassingHoodAngle);
                s_intake.requestSetpoint(autoSHOOTIntakeVoltage.getAsDouble(), 75);
                s_rollers.requestVoltage(autoSHOOTRollersVoltage.getAsDouble());
                s_shooter.requestVelocity(interpolatedPassingShooterVelocity);
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.5) {
                    setState(SuperstructureStates.AUTO_PASS_B);
                }
                break;

            case AUTO_PASS_B:
                s_hood.requestSetpoint(interpolatedPassingHoodAngle);
                s_intake.requestSetpoint(autoSHOOTIntakeVoltage.getAsDouble(), 135);
                s_rollers.requestVoltage(autoSHOOTRollersVoltage.getAsDouble());
                s_shooter.requestVelocity(interpolatedPassingShooterVelocity);
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.5) {
                    setState(SuperstructureStates.AUTO_PASS_A);
                }
                break;

            default:
                break;
        }
    }

    // -------------------------------------------------------------------------
    // Request methods
    // -------------------------------------------------------------------------
    public void requestIdle()          { setState(SuperstructureStates.IDLE); }
    public void requestBump()          { setState(SuperstructureStates.BUMP); }
    public void requestSPITOUT()       { setState(SuperstructureStates.SPITOUT); }
    public void requestUnJam()         { setState(SuperstructureStates.UN_JAM); }
    public void requestSpinUpToShoot() { setState(SuperstructureStates.SPIN_UP); }

    public void requestIntake() {
        if (systemState != SuperstructureStates.AUTO_SPIN_UP
                && systemState != SuperstructureStates.AUTO_SHOOT_A
                && systemState != SuperstructureStates.AUTO_SHOOT_B
                && systemState != SuperstructureStates.AUTO_PASS_SPIN_UP
                && systemState != SuperstructureStates.AUTO_PASS_A
                && systemState != SuperstructureStates.AUTO_PASS_B) {
            setState(SuperstructureStates.INTAKE);
        }
    }

    public void requestAUTOSpinUp() {
        if (systemState != SuperstructureStates.AUTO_SPIN_UP
                && systemState != SuperstructureStates.AUTO_SHOOT_A
                && systemState != SuperstructureStates.AUTO_SHOOT_B) {
            setState(SuperstructureStates.AUTO_SPIN_UP);
        }
    }

    public void requestAUTOPass() {
        if (systemState != SuperstructureStates.AUTO_PASS_SPIN_UP
                && systemState != SuperstructureStates.AUTO_PASS_A
                && systemState != SuperstructureStates.AUTO_PASS_B) {
            setState(SuperstructureStates.AUTO_PASS_SPIN_UP);
        }
    }

    public void setLookaheadDistance(double meters) { this.lookaheadDistanceOverride = meters; }
    public void clearLookaheadDistance()            { this.lookaheadDistanceOverride = null; }

    public void setState(SuperstructureStates nextState) {
        systemState = nextState;
        stateStartTime = RobotController.getFPGATime() / 1E6;
    }

    public void setPreSpin(boolean enabled) {
        preSpinEnabled = enabled;
    }

    private void handleIdleShooter() {
        if (preSpinEnabled) {
            s_shooter.requestMMVelocity(minVelocity.getAsDouble());
        } else {
            s_shooter.requestIdle();
        }
    }

    public boolean isPreSpinEnabled() { return preSpinEnabled; }
    public SuperstructureStates getState() { return systemState; }
}