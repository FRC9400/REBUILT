package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.commons.LoggedTunableNumber;
import frc.commons.ShootingInterpolator;
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

    LoggedTunableNumber SPITOUTintakeVoltage = new LoggedTunableNumber("Superstructure/SPITOUT Intake Voltage", 7);
    LoggedTunableNumber SPITOUTrollersVoltage = new LoggedTunableNumber("Superstructure/SPITOUT Rollers Voltage", -6);

    LoggedTunableNumber INTAKE2intakeVoltage = new LoggedTunableNumber("Superstructure/INTAKE 2 Intake Voltage", 8);
    LoggedTunableNumber INTAKE2shooterVoltage = new LoggedTunableNumber("Superstructure/INTAKE 2 Shooter Voltage", -2);

    LoggedTunableNumber UNJAMrollersVoltage = new LoggedTunableNumber("Superstructure/UNJAM Rollers Voltage", -8);
    LoggedTunableNumber UNJAMshooterVoltage = new LoggedTunableNumber("Superstructure/UNJAM Shooter Voltage", -4);

    LoggedTunableNumber hoodsetpoint = new LoggedTunableNumber("Superstructure/SPINUP AND SHOOT Hood Setpoint Deg", 25);
    LoggedTunableNumber shooterVelocity = new LoggedTunableNumber("Superstructure/SPINUP AND SHOOT Shooter Velocity", 17.5);
    LoggedTunableNumber SHOOTRollersVoltage = new LoggedTunableNumber("Superstructure/SHOOT Rollers Voltage", 8);
    LoggedTunableNumber SHOOTIntakeVoltage = new LoggedTunableNumber("Superstructure/SHOOT Intake Voltage", 2);

    private final DoubleSupplier distanceSupplier;
    private final DoubleSupplier radialVelocitySupplier;

    public Superstructure(HoodIO hoodIO, IntakeIO intakeIO, RollersIO rollersIO, ShooterIO shooterIO, 
            DoubleSupplier distanceSupplier, DoubleSupplier radialVelocitySupplier) {
        this.s_hood = new Hood(hoodIO);
        this.s_intake = new Intake(intakeIO)                                                                                          ;
        this.s_rollers = new Rollers(rollersIO);
        this.s_shooter = new Shooter(shooterIO);
        this.distanceSupplier = distanceSupplier;
        this.radialVelocitySupplier = radialVelocitySupplier;
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
        PIVOT_SHAKING
    }

    @Override
    public void periodic(){
        s_hood.Loop();
        s_intake.Loop();
        s_rollers.Loop();
        s_shooter.Loop();
        double distance = distanceSupplier.getAsDouble();
        Logger.recordOutput("SuperstructureState", this.systemState);
        Logger.recordOutput("State start time", stateStartTime);
        Logger.recordOutput("Superstructure/DistanceToHub", distance);
        Logger.recordOutput("Superstructure/InterpolatedHoodAngle", ShootingInterpolator.getHoodAngle(distance));
        Logger.recordOutput("Superstructure/InterpolatedShooterVelocity", ShootingInterpolator.getShooterVelocity(distance));
        Logger.recordOutput("Superstructure/Radial Velocity", radialVelocitySupplier.getAsDouble());
        switch(systemState){
            case IDLE:
                s_hood.requestIdle();
                s_intake.requestIdle();
                s_rollers.requestIdle();
                s_shooter.requestIdle();
                break;
            case BUMP:
                s_hood.requestIdle();
                s_intake.requestRaised();
                s_rollers.requestIdle();
                s_shooter.requestIdle();
                break;
            case INTAKE:
                s_hood.requestIdle();
                s_intake.requestIntake(6);
                s_rollers.requestIdle();
                s_shooter.requestIdle();
                break;
            case INTAKE_2A:
                s_hood.requestIdle();
                s_intake.requestIntake(INTAKE2intakeVoltage.getAsDouble());
                s_rollers.requestIdle();
                s_shooter.requestVoltage(INTAKE2shooterVoltage.getAsDouble());
                break;
            case INTAKE_2B:
                s_hood.requestIdle();
                s_intake.requestLowered();
                s_rollers.requestIdle();
                s_shooter.requestVoltage(INTAKE2shooterVoltage.getAsDouble());
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 1){
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case SPITOUT:
                s_hood.requestIdle();
                s_intake.requestIntake(-SPITOUTintakeVoltage.getAsDouble());
                s_rollers.requestVoltage(SPITOUTrollersVoltage.getAsDouble());
                s_shooter.requestIdle();
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
                s_shooter.requestVelocity(shooterVelocity.getAsDouble());
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.2){
                    setState(SuperstructureStates.SHOOT_A);
                }
                break;
            case SHOOT_A:
                s_hood.requestSetpoint(hoodsetpoint.getAsDouble());
                s_intake.requestSetpoint(SHOOTIntakeVoltage.getAsDouble(), 70);
                s_rollers.requestVoltage(SHOOTRollersVoltage.getAsDouble());
                s_shooter.requestVelocity(shooterVelocity.getAsDouble());
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.5){
                    setState(SuperstructureStates.SHOOT_B);
                }
                break;
            case SHOOT_B: 
                s_hood.requestSetpoint(hoodsetpoint.getAsDouble());
                s_intake.requestSetpoint(SHOOTIntakeVoltage.getAsDouble(), 115);
                s_rollers.requestVoltage(SHOOTRollersVoltage.getAsDouble());
                s_shooter.requestVelocity(shooterVelocity.getAsDouble());
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.5){
                    setState(SuperstructureStates.SHOOT_A);
                }
                break;
            case AUTO_SPIN_UP:
                s_intake.requestLowered();
                s_rollers.requestIdle();
                s_shooter.requestVelocity(ShootingInterpolator.getShooterVelocity(distance) - radialVelocitySupplier.getAsDouble() + 0.17);
                s_hood.requestSetpoint(ShootingInterpolator.getHoodAngle(distance));
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 1.5) {
                    setState(SuperstructureStates.AUTO_SHOOT_A);
                }
                break;
            case AUTO_SHOOT_A:
                s_hood.requestSetpoint(ShootingInterpolator.getHoodAngle(distance));
                s_intake.requestSetpoint(SHOOTRollersVoltage.getAsDouble(), 70);
                s_rollers.requestVoltage(SHOOTRollersVoltage.getAsDouble());
                s_shooter.requestVelocity(ShootingInterpolator.getShooterVelocity(distance) - radialVelocitySupplier.getAsDouble() - 0.05);
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.5) {
                    setState(SuperstructureStates.AUTO_SHOOT_B);
                }
                break;
            case AUTO_SHOOT_B:
                s_hood.requestSetpoint(ShootingInterpolator.getHoodAngle(distance));
                s_intake.requestSetpoint(SHOOTRollersVoltage.getAsDouble(), 115);
                s_rollers.requestVoltage(SHOOTRollersVoltage.getAsDouble());
                s_shooter.requestVelocity(ShootingInterpolator.getShooterVelocity(distance) - radialVelocitySupplier.getAsDouble() - 0.05);
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.5) {
                    setState(SuperstructureStates.AUTO_SHOOT_A);
                }
                break;
            default:
                break;
        }
    }

    public void requestIdle(){
        setState(SuperstructureStates.IDLE);
    }

    public void requestBump(){
        setState(SuperstructureStates.BUMP);
    }

    public void requestIntake(){
        if(systemState != SuperstructureStates.AUTO_SPIN_UP && systemState != SuperstructureStates.AUTO_SHOOT_A && systemState != SuperstructureStates.AUTO_SHOOT_B){
            setState(SuperstructureStates.INTAKE);
        }
    }

    public void requestSPITOUT(){
        setState(SuperstructureStates.SPITOUT);
    }

    public void requestUnJam(){
        setState(SuperstructureStates.UN_JAM);
    }

    public void requestSpinUpToShoot(){
        setState(SuperstructureStates.SPIN_UP);
    }

    public void requestAUTOSpinUp(){
        if(systemState != SuperstructureStates.AUTO_SPIN_UP && systemState != SuperstructureStates.AUTO_SHOOT_A && systemState != SuperstructureStates.AUTO_SHOOT_B){
            setState(SuperstructureStates.AUTO_SPIN_UP);
        }
    }

    public void setState(SuperstructureStates nextState){
        systemState = nextState;
        stateStartTime = RobotController.getFPGATime() / 1E6;
    }

    public SuperstructureStates getState(){
        return systemState;
    }
}