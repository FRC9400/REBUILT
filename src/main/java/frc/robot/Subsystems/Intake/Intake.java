package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static edu.wpi.first.units.Units.Volts;

public class Intake extends SubsystemBase {
    private final IntakeIO intakeIO;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final SysIdRoutine pivotRoutine;

    public Intake(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;
        pivotRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(4), null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> intakeIO.requestPivotVoltage(volts.in(Volts)), null, this));
    }

    public Command intakeSysIdCmd(){
        return Commands.sequence(
            this.runOnce(() -> SignalLogger.start()),
            pivotRoutine
                    .quasistatic(Direction.kForward)
                    .until(() -> Math.abs(inputs.pivotPosDeg) > 90), 
            this.runOnce(() -> intakeIO.requestPivotVoltage(0)),
            Commands.waitSeconds(1),
            pivotRoutine
                    .quasistatic(Direction.kReverse)
                    .until(() -> inputs.pivotPosDeg < 5), 
            this.runOnce(() -> intakeIO.requestPivotVoltage(0)),
            Commands.waitSeconds(1),

            pivotRoutine
                    .dynamic(Direction.kForward)
                    .until(() -> Math.abs(inputs.pivotPosDeg) > 90),
            this.runOnce(() -> intakeIO.requestPivotVoltage(0)),
            Commands.waitSeconds(1),

            pivotRoutine
                    .dynamic(Direction.kReverse)
                    .until(() -> inputs.pivotPosDeg < 5),
            this.runOnce(() -> intakeIO.requestPivotVoltage(0)),
            Commands.waitSeconds(1),
            this.runOnce(() -> SignalLogger.stop()));
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
