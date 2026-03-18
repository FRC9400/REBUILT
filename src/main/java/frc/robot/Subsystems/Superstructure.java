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

    LoggedTunableNumber INTAKEintakeVoltage = new LoggedTunableNumber("Superstructure/INTAKE Intake Voltage", 11);
    LoggedTunableNumber OUTTAKErollersVoltage = new LoggedTunableNumber("Superstructure/OUTTAKE Rollers Voltage", -3);

    LoggedTunableNumber INTAKE2rollersVoltage = new LoggedTunableNumber("Superstructure/INTAKE 2 Rollers Voltage", 3);
    LoggedTunableNumber INTAKE2intakeVoltage = new LoggedTunableNumber("Superstructure/INTAKE 2 Intake Voltage", 8);
    LoggedTunableNumber INTAKE2shooterVoltage = new LoggedTunableNumber("Superstructure/INTAKE 2 Shooter Voltage", -2);

    LoggedTunableNumber UNJAMrollersVoltage = new LoggedTunableNumber("Superstructure/UNJAM Rollers Voltage", -8);
    LoggedTunableNumber UNJAMshooterVoltage = new LoggedTunableNumber("Superstructure/UNJAM Shooter Voltage", -4);

    LoggedTunableNumber hoodsetpoint = new LoggedTunableNumber("Superstructure/SPINUP AND SHOOT Hood Setpoint Deg", 4);
    LoggedTunableNumber shooterVelocity = new LoggedTunableNumber("Superstructure/SPINUP AND SHOOT Shooter Velocity", 17.5);
    LoggedTunableNumber SHOOTRollersVelocity = new LoggedTunableNumber("Superstructure/SHOOT Rollers Velocity", 8);
    LoggedTunableNumber SHOOTIntakeVoltage = new LoggedTunableNumber("Superstructure/SHOOT Intake Voltage", 2);

    private final DoubleSupplier distanceSupplier;
    private final DoubleSupplier radialVelocitySupplier;

    public Superstructure(HoodIO hoodIO, IntakeIO intakeIO, RollersIO rollersIO, ShooterIO shooterIO, 
            DoubleSupplier distanceSupplier, DoubleSupplier radialVelocitySupplier) {
        this.s_hood = new Hood(hoodIO);
        this.s_intake = new Intake(intakeIO);
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
        OUTTAKE,
        UN_JAM,
        SPIN_UP,
        SHOOT,
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
                s_intake.requestIntake(3);
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
            case OUTTAKE:
                s_hood.requestIdle();
                s_intake.requestIntake(-INTAKEintakeVoltage.getAsDouble());
                s_rollers.requestVoltage(OUTTAKErollersVoltage.getAsDouble());
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
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 1){
                    setState(SuperstructureStates.SHOOT);
                }
                break;
            case SHOOT:
                s_hood.requestSetpoint(hoodsetpoint.getAsDouble());
                s_intake.requestIntake(SHOOTIntakeVoltage.getAsDouble());
                s_rollers.requestVoltage(SHOOTRollersVelocity.getAsDouble());
                s_shooter.requestVelocity(shooterVelocity.getAsDouble());
                break;
            case AUTO_SPIN_UP:
                s_hood.requestSetpoint(ShootingInterpolator.getHoodAngle(distance));
                s_intake.requestLowered();
                s_rollers.requestIdle();
                s_shooter.requestVelocity(ShootingInterpolator.getShooterVelocity(distance) - radialVelocitySupplier.getAsDouble() - 0.3);
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 1) {
                    setState(SuperstructureStates.AUTO_SHOOT_A);
                }
                break;
            case AUTO_SHOOT_A:
                s_hood.requestSetpoint(ShootingInterpolator.getHoodAngle(distance));
                s_intake.requestSetpoint(SHOOTRollersVelocity.getAsDouble(), 85);
                s_rollers.requestVoltage(SHOOTRollersVelocity.getAsDouble());
                s_shooter.requestVelocity(ShootingInterpolator.getShooterVelocity(distance) - radialVelocitySupplier.getAsDouble() - 0.3);
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.5) {
                    setState(SuperstructureStates.AUTO_SHOOT_B);
                }
                break;
            case AUTO_SHOOT_B:
                s_hood.requestSetpoint(ShootingInterpolator.getHoodAngle(distance));
                s_intake.requestSetpoint(SHOOTRollersVelocity.getAsDouble(), 105);
                s_rollers.requestVoltage(SHOOTRollersVelocity.getAsDouble());
                s_shooter.requestVelocity(ShootingInterpolator.getShooterVelocity(distance) - radialVelocitySupplier.getAsDouble() - 0.3);
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
        setState(SuperstructureStates.INTAKE);
    }

    public void requestOuttake(){
        setState(SuperstructureStates.OUTTAKE);
    }

    public void requestUnJam(){
        setState(SuperstructureStates.UN_JAM);
    }

    public void requestSpinUpToShoot(){
        setState(SuperstructureStates.SPIN_UP);
    }

    public void requestAUTOSpinUp(){
        setState(SuperstructureStates.AUTO_SPIN_UP);
    }

    public void setState(SuperstructureStates nextState){
        systemState = nextState;
        stateStartTime = RobotController.getFPGATime() / 1E6;
    }

    public SuperstructureStates getState(){
        return systemState;
    }
}
