package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

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

    LoggedTunableNumber SPITOUTintakeVoltage  = new LoggedTunableNumber("Superstructure/SPITOUT Intake Voltage", 7);
    LoggedTunableNumber SPITOUTrollersVoltage = new LoggedTunableNumber("Superstructure/SPITOUT Rollers Voltage", -6);

    LoggedTunableNumber INTAKE2intakeVoltage  = new LoggedTunableNumber("Superstructure/INTAKE 2 Intake Voltage", 8);
    LoggedTunableNumber INTAKE2shooterVoltage = new LoggedTunableNumber("Superstructure/INTAKE 2 Shooter Voltage", -2);

    LoggedTunableNumber UNJAMrollersVoltage = new LoggedTunableNumber("Superstructure/UNJAM Rollers Voltage", -8);
    LoggedTunableNumber UNJAMshooterVoltage = new LoggedTunableNumber("Superstructure/UNJAM Shooter Voltage", -4);

    LoggedTunableNumber hoodsetpoint     = new LoggedTunableNumber("Superstructure/SPINUP AND SHOOT Hood Setpoint Deg", 25);
    LoggedTunableNumber shooterVelocity  = new LoggedTunableNumber("Superstructure/SPINUP AND SHOOT Shooter Velocity", 18);
    LoggedTunableNumber SHOOTRollersVoltage = new LoggedTunableNumber("Superstructure/SHOOT Rollers Voltage", 8);
    LoggedTunableNumber SHOOTIntakeVoltage  = new LoggedTunableNumber("Superstructure/SHOOT Intake Voltage", 2);

    LoggedTunableNumber minVelocity = new LoggedTunableNumber("Superstructure/MIN VELOCITY FOR SPINUP", 12.7);

    // distanceSuppliers are only
    // used when LaunchCalculator lookahead is not active
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
        INTAKE_2A,
        INTAKE_2B,
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

        // Use lookahead distance when ShootOnMoveCommand/PassOnMoveCommand is active,
        // fall back to raw distance otherwise
        double distance = lookaheadDistanceOverride != null
                ? lookaheadDistanceOverride
                : distanceSupplier.getAsDouble();

        double passingDistance = lookaheadDistanceOverride != null
            ? lookaheadDistanceOverride
            : distanceToPassTargetSupplier.getAsDouble();

        Logger.recordOutput("SuperstructureState", this.systemState);
        Logger.recordOutput("Superstructure/DistanceToHub", distance);
        Logger.recordOutput("Superstructure/UsingLookahead", lookaheadDistanceOverride != null);
        Logger.recordOutput("Superstructure/InterpolatedHoodAngle", LaunchCalculator.getHoodAngleDeg(distance));
        Logger.recordOutput("Superstructure/InterpolatedShooterVelocity", LaunchCalculator.getShooterVelocity(distance));

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
                s_intake.requestIntake(8);
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
                s_hood.requestSetpoint(hoodsetpoint.getAsDouble());
                s_intake.requestLowered();
                s_rollers.requestIdle();
                s_shooter.requestMMVelocity(shooterVelocity.getAsDouble());
                if (s_shooter.atSetpoint()) {
                    setState(SuperstructureStates.SHOOT_A);
                }
                break;

            case SHOOT_A:
                s_hood.requestSetpoint(hoodsetpoint.getAsDouble());
                s_intake.requestSetpoint(SHOOTIntakeVoltage.getAsDouble(), 70);
                s_rollers.requestVoltage(SHOOTRollersVoltage.getAsDouble());
                s_shooter.requestMMVelocity(shooterVelocity.getAsDouble());
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.5) {
                    setState(SuperstructureStates.SHOOT_B);
                }
                break;

            case SHOOT_B:
                s_hood.requestSetpoint(hoodsetpoint.getAsDouble());
                s_intake.requestSetpoint(SHOOTIntakeVoltage.getAsDouble(), 105);
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
                s_hood.requestSetpoint(LaunchCalculator.getHoodAngleDeg(distance));
                s_shooter.requestMMVelocity(LaunchCalculator.getShooterVelocity(distance));
                if (s_shooter.atSetpoint()) {
                    setState(SuperstructureStates.AUTO_SHOOT_A);
                }
                break;

            case AUTO_SHOOT_A:
                s_hood.requestSetpoint(LaunchCalculator.getHoodAngleDeg(distance));
                s_intake.requestSetpoint(SHOOTRollersVoltage.getAsDouble(), 70);
                s_rollers.requestVoltage(SHOOTRollersVoltage.getAsDouble());
                s_shooter.requestMMVelocity(LaunchCalculator.getShooterVelocity(distance));
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.5) {
                    setState(SuperstructureStates.AUTO_SHOOT_B);
                }
                break;

            case AUTO_SHOOT_B:
                s_hood.requestSetpoint(LaunchCalculator.getHoodAngleDeg(distance));
                s_intake.requestSetpoint(SHOOTRollersVoltage.getAsDouble(), 105);
                s_rollers.requestVoltage(SHOOTRollersVoltage.getAsDouble());
                s_shooter.requestMMVelocity(LaunchCalculator.getShooterVelocity(distance));
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
                s_hood.requestSetpoint(LaunchCalculator.getPassingHoodAngleDeg(passingDistance));
                s_shooter.requestVelocity(LaunchCalculator.getPassingShooterVelocity(passingDistance));
                if (s_shooter.atSetpoint()) {
                    setState(SuperstructureStates.AUTO_PASS_A);
                }
                break;

            case AUTO_PASS_A:
                s_hood.requestSetpoint(LaunchCalculator.getPassingHoodAngleDeg(passingDistance));
                s_intake.requestSetpoint(SHOOTRollersVoltage.getAsDouble(), 70);
                s_rollers.requestVoltage(SHOOTRollersVoltage.getAsDouble());
                s_shooter.requestVelocity(LaunchCalculator.getPassingShooterVelocity(passingDistance));
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.5) {
                    setState(SuperstructureStates.AUTO_PASS_B);
                }
                break;

            case AUTO_PASS_B:
                s_hood.requestSetpoint(LaunchCalculator.getPassingHoodAngleDeg(passingDistance));
                s_intake.requestSetpoint(SHOOTRollersVoltage.getAsDouble(), 105);
                s_rollers.requestVoltage(SHOOTRollersVoltage.getAsDouble());
                s_shooter.requestVelocity(LaunchCalculator.getPassingShooterVelocity(passingDistance));
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

    // Lookahead distance override
    public void setLookaheadDistance(double meters) { this.lookaheadDistanceOverride = meters; }
    public void clearLookaheadDistance()            { this.lookaheadDistanceOverride = null; }

    // State management
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

    public boolean isPreSpinEnabled() {
        return preSpinEnabled;
    }

    public SuperstructureStates getState() { return systemState; }
}