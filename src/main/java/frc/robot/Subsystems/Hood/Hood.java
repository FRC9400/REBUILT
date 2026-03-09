package frc.robot.Subsystems.Hood;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import static edu.wpi.first.units.Units.Volts;

public class Hood extends SubsystemBase{
    private final HoodIO hoodIO;
    private HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
    private double setpointDeg = 0;
    private double setpointVolts = 0;
    private final SysIdRoutine hoodRoutine;

    public Hood(HoodIO hoodIO){
        this.hoodIO = hoodIO;
        hoodRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(4), null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> hoodIO.requestVoltage(volts.in(Volts)), null, this));
    
    }

    public Command hoodSysIdCmd(){
        return Commands.sequence(
            this.runOnce(() -> SignalLogger.start()),
            hoodRoutine
                    .quasistatic(Direction.kForward)
                    .until(() -> Math.abs(hoodInputs.hoodPosDeg) > 13), 
            this.runOnce(() -> hoodIO.requestVoltage(0)),
            Commands.waitSeconds(1),
            hoodRoutine
                    .quasistatic(Direction.kReverse)
                    .until(() -> hoodInputs.hoodPosDeg < 2), 
            this.runOnce(() -> hoodIO.requestVoltage(0)),
            Commands.waitSeconds(1),

            hoodRoutine
                    .dynamic(Direction.kForward)
                    .until(() -> Math.abs(hoodInputs.appliedDeg) > 13),
            this.runOnce(() -> hoodIO.requestVoltage(0)),
            Commands.waitSeconds(1),

            hoodRoutine
                    .dynamic(Direction.kReverse)
                    .until(() -> hoodInputs.appliedDeg < 4),
            this.runOnce(() -> hoodIO.requestVoltage(0)),
            Commands.waitSeconds(1),
            this.runOnce(() -> SignalLogger.stop()));
    }

    @Override
    public void periodic(){
        hoodIO.updateInputs(hoodInputs);
        Logger.processInputs("Hood", hoodInputs);
    }

    public void requestMotionMagic(double setpointDeg){
        hoodIO.requestMotionMagic(setpointDeg);
    }

    public void requestVoltage(double setpointVolts){
        hoodIO.requestVoltage(setpointVolts);
    }

}