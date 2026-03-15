package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.fieldConstants;

public class SnapToHubCommand extends Command {
    private final Swerve swerve;
    private final PIDController thetaController = new PIDController(4, 0, 0.15);

    public SnapToHubCommand(Swerve swerve) {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double x = MathUtil.applyDeadband(-RobotContainer.driver.getLeftY(), 0.1);
        double y = MathUtil.applyDeadband(-RobotContainer.driver.getLeftX(), 0.1);

        Translation2d hubPosition = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? fieldConstants.BLUE_HUB_POS.getTranslation()
            : fieldConstants.RED_HUB_POS.getTranslation();

        Translation2d toHub = hubPosition.minus(swerve.getPoseRaw().getTranslation());
        double targetAngle = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? Math.atan2(toHub.getY(), toHub.getX())
            : Math.atan2(toHub.getY(), toHub.getX()) + Math.PI;

        double thetaFeedback = thetaController.calculate(
            swerve.getGyroPositionRadians(),
            targetAngle
        );
        thetaFeedback = MathUtil.clamp(thetaFeedback, -5, 5);

        swerve.requestDesiredState(x * 4.72, y * 4.72, thetaFeedback, true, false);
    }

    @Override
    public void end(boolean interrupted) {}
}