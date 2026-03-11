package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;

public class AutoShootCommand extends Command {
    private final Superstructure superstructure;
    private double shootTimer = 0;
    private static final double SHOOT_DURATION = 3;

    public AutoShootCommand(Superstructure superstructure) {
        this.superstructure = superstructure;
    }

    @Override
    public void initialize() {
        shootTimer = 0;
        superstructure.requestAUTOSpinUp();
    }

    @Override
    public void execute() {
        if (superstructure.getState() == SuperstructureStates.AUTO_SHOOT) {
            shootTimer += 0.02;
        }
    }

    @Override
    public boolean isFinished() {
        return shootTimer >= SHOOT_DURATION;
    }

    @Override
    public void end(boolean interrupted) {
        superstructure.requestIdle();
    }
}