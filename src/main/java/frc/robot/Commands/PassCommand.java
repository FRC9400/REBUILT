package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.commons.FieldConstants;
import frc.commons.LaunchCalculator;
import frc.robot.RobotContainer;
import frc.robot.Constants.swerveConstants;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Swerve.Swerve;
import org.littletonrobotics.junction.Logger;

public class PassCommand extends Command {
    private final Superstructure superstructure;
    private final Swerve swerve;

    public PassCommand(Swerve swerve, Superstructure superstructure) {
        this.swerve = swerve;
        this.superstructure = superstructure;
    }

    @Override
    public void initialize() {
        superstructure.requestAUTOPass();
    }

    @Override
    public void execute() {
        Translation2d passTarget = LaunchCalculator.getPassingTarget(swerve.getPoseRaw());
        double distance = swerve.getPoseRaw().getTranslation().getDistance(passTarget);
        superstructure.setLookaheadDistance(distance);
    }

    @Override
    public void end(boolean interrupted) {
        superstructure.clearLookaheadDistance();
        superstructure.requestIdle();
    }

    @Override
    public boolean isFinished() { return false; }
}