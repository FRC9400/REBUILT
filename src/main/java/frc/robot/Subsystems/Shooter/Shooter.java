package frc.robot.Subsystems.Shooter;


import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static edu.wpi.first.units.Units.Volts;

public class Shooter extends SubsystemBase{
    private final ShooterIO shooterIO;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private double setpointVelocity;
    private final SysIdRoutine shooterRoutine;

    public Shooter(ShooterIO shooterIO){
        this.shooterIO = shooterIO;
        setpointVelocity = 0.0;
        
        shooterRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(4), null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> shooterIO.requestVoltage(volts.in(Volts)), null, this));
    }

    public Command shooterSysIdCmd(){
        return Commands.sequence(
            this.runOnce(() -> SignalLogger.start()),
            shooterRoutine
                    .quasistatic(Direction.kForward)
                    .until(() -> inputs.shooterVelMPS[0] > 10), 
            this.runOnce(() -> shooterIO.requestVoltage(0)),
            Commands.waitSeconds(1),
            shooterRoutine
                    .quasistatic(Direction.kReverse)
                    .until(() -> inputs.shooterVelMPS[0] < -10), 
            this.runOnce(() -> shooterIO.requestVoltage(0)),
            Commands.waitSeconds(1),
            shooterRoutine
                    .dynamic(Direction.kForward)
                    .until(() -> inputs.shooterVelMPS[0] > 10),
            this.runOnce(() -> shooterIO.requestVoltage(0)),
            Commands.waitSeconds(1),
            shooterRoutine
                    .dynamic(Direction.kReverse)
                    .until(() -> inputs.shooterVelMPS[0] < -10), //Keep in mind the max height is around 0.6
            this.runOnce(() -> shooterIO.requestVoltage(0)),
            Commands.waitSeconds(1),
            this.runOnce(() -> SignalLogger.stop()));
    }

    @Override
    public void periodic(){
        shooterIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public void setVelocity(double velocity, double ratio) {
        setpointVelocity = velocity;
        shooterIO.setVelocity(setpointVelocity, ratio);
    }

    public void zeroVelocity() {
        setpointVelocity = 0;
        shooterIO.zeroVelocity();
    }

    public void requestVoltage(double volts){
        shooterIO.requestVoltage(volts);
    }
}