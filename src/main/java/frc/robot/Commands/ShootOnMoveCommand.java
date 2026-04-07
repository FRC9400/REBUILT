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

public class ShootOnMoveCommand extends Command {
    private final Swerve swerve;
    private final Superstructure superstructure;
    private final boolean passing;

    // TUNE: rotation PID gains for hub-lock
    private static final double driveLaunchKp = 6;
    private static final double driveLaunchKd = 0;

    // TUNE: max angular velocity (rad/s) of the lookahead pose relative to hub.
    // Lower means more conservative speed limiting while shooting.
    private static final double maxPolarVelocityRadPerSec = 0.8;

    public ShootOnMoveCommand(Swerve swerve, Superstructure superstructure, boolean passing) {
        this.swerve = swerve;
        this.superstructure = superstructure;
        this.passing = passing;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        LaunchCalculator.getInstance().clearLaunchingParameters();
        if (passing) {
            superstructure.requestAUTOPass();
        } else {
            superstructure.requestAUTOSpinUp();
        }
    }

    @Override
    public void execute() {
        LaunchCalculator.getInstance().clearLaunchingParameters();

        Translation2d hubTarget = getHubTarget();

        ChassisSpeeds robotRelativeSpeeds = swerve.getRobotRelativeSpeeds();
        ChassisSpeeds fieldRelativeSpeeds = swerve.getFieldRelativeSpeeds().times(-1);

        var parameters = LaunchCalculator.getInstance().getParameters(
                swerve.getPoseRaw(),
                robotRelativeSpeeds,
                fieldRelativeSpeeds,
                hubTarget,
                passing);

        superstructure.setLookaheadDistance(parameters.distance());

        // Rotation: feedforward + P on angle error + D on omega error
        double angleError = parameters.driveAngle()
                .minus(swerve.getRotation2d())
                .getRadians();
        double omegaError = parameters.driveVelocity()
                - robotRelativeSpeeds.omegaRadiansPerSecond;
        double omegaOutput = /*parameters.driveVelocity()
                +*/ MathUtil.clamp(((angleError * driveLaunchKp)
                + (omegaError * driveLaunchKd)), -10, 10);

        // Joystick linear velocity
        double x = MathUtil.applyDeadband(RobotContainer.driver.getLeftY(), 0.1);
        double y = MathUtil.applyDeadband(RobotContainer.driver.getLeftX(), 0.1);
        Translation2d linearVelocity = new Translation2d(
                x * swerveConstants.moduleConstants.maxSpeedMeterPerSecond * 0.5,
                y * swerveConstants.moduleConstants.maxSpeedMeterPerSecond * 0.5);

        // Flip for red alliance
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            linearVelocity = linearVelocity.times(-1.0);
        }

        // Polar velocity limiter: prevent lookahead from drifting too far angularly
        double robotHubDistance = parameters.distanceNoLookahead();
        double naiveTOF = LaunchCalculator.getInstance().getNaiveTOF(robotHubDistance);
        double hubAngle = maxPolarVelocityRadPerSec * naiveTOF;

        if (linearVelocity.getNorm() > 1e-6) {
            double robotAngle = Math.abs(
                    hubTarget.minus(swerve.getPoseRaw().getTranslation())
                            .getAngle()
                            .minus(linearVelocity.getAngle())
                            .getRadians());
            double lookaheadAngle = Math.PI - robotAngle - hubAngle; 
            if (lookaheadAngle > 0) {
                double maxLinearVelocity =
                        robotHubDistance * Math.sin(hubAngle) / Math.sin(lookaheadAngle) / naiveTOF;
                if (linearVelocity.getNorm() > maxLinearVelocity) {
                    linearVelocity = linearVelocity.times(maxLinearVelocity / linearVelocity.getNorm());
                }
            }
        }

        swerve.requestDesiredState(
                linearVelocity.getX(),
                linearVelocity.getY(),
                omegaOutput,
                true,
                false);

        // Logging
        Logger.recordOutput("ShootOnMove/IsValid", parameters.isValid());
        Logger.recordOutput("ShootOnMove/AngleErrorDeg", Math.toDegrees(angleError));
        Logger.recordOutput("ShootOnMove/DriveAngleDeg", parameters.driveAngle().getDegrees());
        Logger.recordOutput("ShootOnMove/LookaheadDistance", parameters.distance());
    }

    @Override
    public void end(boolean interrupted) {
        superstructure.clearLookaheadDistance();
        superstructure.requestIdle();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public static Translation2d getHubTarget() {
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        Translation2d blueHub = FieldConstants.Hub.topCenterPoint.toTranslation2d();
        // Red hub is mirrored across field length
        return isRed
                ? new Translation2d(FieldConstants.fieldLength - blueHub.getX(), blueHub.getY())
                : blueHub;
    }
}