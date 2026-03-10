package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Superstructure;

public class IntakeCommand extends Command {
    private final Superstructure superstructure;

    public IntakeCommand(Superstructure superstructure) {
        this.superstructure = superstructure;
    }

    @Override
    public void initialize() {
        superstructure.requestIntake();
    }

    @Override
    public void end(boolean interrupted) {
        superstructure.requestIdle(); 
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}