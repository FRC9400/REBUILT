package frc.commons;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.annotation.Target;

import org.littletonrobotics.junction.Logger;

public class LaunchCalculator {
    private static LaunchCalculator instance;



    public static LaunchCalculator getInstance() {
        if (instance == null) instance = new LaunchCalculator();
        return instance;
    }
// Robot geometry
    // TUNE: x = forward offset of launcher from robot center (m)
    //       y = lateral offset (m, positive = left)
    //       rotation = direction launcher faces relative to robot front
    // nums from CAD
    public static final Transform2d robotToLauncher =
            new Transform2d(new Translation2d(-0.0813, 0.0), Rotation2d.fromDegrees(0.0));

    private static final double phaseDelay = 0.03;  // TUNE: seconds
    private static final double loopPeriodSecs = 0.02;

    private static final double minDistance = 1.0;   // TUNE
    private static final double maxDistance = 5.5;   // TUNE

    // TUNE: passing target on field (blue-alliance origin, meters)
    private static final double xPassTarget = 1.85;  // TUNE
    private static final double yPassTarget = 1.65;  // TUNE
    private static final double passingMinDistance = 4.0;   // TUNE
    private static final double passingMaxDistance = 12.0;  // TUNE

    // -------------------------------------------------------------------------
    // Shooting lookup maps
    // -------------------------------------------------------------------------
    private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private static final InterpolatingDoubleTreeMap shooterVelocityMap =
            new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap timeOfFlightMap =
            new InterpolatingDoubleTreeMap();

    // -------------------------------------------------------------------------
    // Passing lookup maps
    // -------------------------------------------------------------------------
    private static final InterpolatingTreeMap<Double, Rotation2d> passingHoodAngleMap =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private static final InterpolatingDoubleTreeMap passingShooterVelocityMap =
            new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap passingTimeOfFlightMap =
            new InterpolatingDoubleTreeMap();

    static {
        // TUNE: distance (m) → hood angle (deg), shooter velocity (mps), time of flight (s)
        hoodAngleMap.put(1.68,  Rotation2d.fromDegrees(0.0));
        hoodAngleMap.put(1.977, Rotation2d.fromDegrees(0.05));
        hoodAngleMap.put(2.28,  Rotation2d.fromDegrees(0.1));
        hoodAngleMap.put(2.611, Rotation2d.fromDegrees(0.18));
        hoodAngleMap.put(3.089, Rotation2d.fromDegrees(0.25));
        hoodAngleMap.put(3.454, Rotation2d.fromDegrees(0.4));
        hoodAngleMap.put(3.809, Rotation2d.fromDegrees(0.65));
        hoodAngleMap.put(4.38,  Rotation2d.fromDegrees(0.78));
        hoodAngleMap.put(4.89,  Rotation2d.fromDegrees(0.9));

        shooterVelocityMap.put(1.68,  12.7);
        shooterVelocityMap.put(1.977, 13.4);
        shooterVelocityMap.put(2.28,  13.8);
        shooterVelocityMap.put(2.611, 14.8);
        shooterVelocityMap.put(3.089, 15.2);
        shooterVelocityMap.put(3.454, 15.8);
        shooterVelocityMap.put(3.809, 17.0);
        shooterVelocityMap.put(4.38,  18.6);
        shooterVelocityMap.put(4.89,  19.6);

        timeOfFlightMap.put(1.68,  0.92);
        timeOfFlightMap.put(1.977, 1.0);
        timeOfFlightMap.put(2.28,  1.06);
        timeOfFlightMap.put(2.611, 1.19);
        timeOfFlightMap.put(3.089, 1.16);
        timeOfFlightMap.put(3.454, 1.21);
        timeOfFlightMap.put(3.809, 1.28);
        timeOfFlightMap.put(4.38,  1.46);
        timeOfFlightMap.put(4.89,  1.39);

        // TUNE: passing maps
        // Hood angle is typically flat since speed scales with distance but idk
        passingHoodAngleMap.put(4.0,  Rotation2d.fromDegrees(0.0));
        passingHoodAngleMap.put(6.0,  Rotation2d.fromDegrees(0.0));
        passingHoodAngleMap.put(8.0,  Rotation2d.fromDegrees(0.0));
        passingHoodAngleMap.put(10.0, Rotation2d.fromDegrees(0.0));
        passingHoodAngleMap.put(12.0, Rotation2d.fromDegrees(0.0));

        passingShooterVelocityMap.put(4.0,  0.0);
        passingShooterVelocityMap.put(6.0,  0.0);
        passingShooterVelocityMap.put(8.0,  0.0);
        passingShooterVelocityMap.put(10.0, 0.0);
        passingShooterVelocityMap.put(12.0, 0.0);

        passingTimeOfFlightMap.put(4.0,  0.0);
        passingTimeOfFlightMap.put(6.0,  0.0);
        passingTimeOfFlightMap.put(8.0,  0.0);
        passingTimeOfFlightMap.put(10.0, 0.0);
        passingTimeOfFlightMap.put(12.0, 0.0);
    }

    // -------------------------------------------------------------------------
    // LaunchingParameters record
    // -------------------------------------------------------------------------
    public record LaunchingParameters(
            boolean isValid,
            Rotation2d driveAngle,
            double driveVelocity,
            double hoodAngleDeg,
            double hoodVelocity,
            double shooterVelocity,
            double distance,
            double distanceNoLookahead,
            double timeOfFlight,
            boolean passing
    ) {}

    // -------------------------------------------------------------------------
    // Filters and state
    // -------------------------------------------------------------------------
    private static final int filterTaps = (int) (0.1 / loopPeriodSecs); // average rate of change over the last 100ms

    private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage(filterTaps);
    private final LinearFilter driveAngleFilter = LinearFilter.movingAverage(filterTaps);

    private double lastHoodAngleDeg = Double.NaN;
    private Rotation2d lastDriveAngle = null;

    private Field2d m_field = new Field2d();

    private double hoodAngleOffsetDeg = 0.0;

    // Setpoint velocity for auto
    private ChassisSpeeds fieldSetpointVelocity = new ChassisSpeeds();

    private LaunchingParameters latestParameters = null;

    // Setpoint velocity (for auto accuracy use commanded speed)
    public void setFieldSetpointVelocity(ChassisSpeeds speeds) {
        this.fieldSetpointVelocity = speeds;
    }

    // -------------------------------------------------------------------------
    // Main calculation
    // -------------------------------------------------------------------------
    public LaunchingParameters getParameters(
            Pose2d estimatedPose,
            ChassisSpeeds robotVelocity,
            ChassisSpeeds fieldVelocity,
            Translation2d hubTarget,
            boolean passing) {

        if (latestParameters != null) return latestParameters;

        // In auto, use setpoint velocity instead of measured 
        ChassisSpeeds velocityForLookahead = DriverStation.isAutonomous()
                ? fieldSetpointVelocity
                : transformVelocityToLauncher(fieldVelocity, robotVelocity, estimatedPose.getRotation());

        // Phase delay: nudge estimated pose forward in time
        Pose2d phasePose = estimatedPose.exp(new Twist2d(
                robotVelocity.vxMetersPerSecond * phaseDelay, //JOSIE CHANGE THESE TO NEGATIVE
                robotVelocity.vyMetersPerSecond * phaseDelay,
                robotVelocity.omegaRadiansPerSecond * phaseDelay));

        // Launcher position on field
        Pose2d launcherPose = phasePose.transformBy(robotToLauncher);
        Translation2d launcherTranslation = launcherPose.getTranslation();

        // Target: hub for shooting, fixed field point for passing
        Translation2d target = passing ? getPassingTarget(estimatedPose) : hubTarget;

        double rawDistance = target.getDistance(launcherTranslation);

        // Iterative lookahead 
        double tof = getTOF(passing, rawDistance);
        double lookaheadDist = rawDistance;
        Translation2d lookaheadLauncherPos = launcherTranslation;

        for (int i = 0; i < 20; i++) {
            tof = getTOF(passing, lookaheadDist);
            lookaheadLauncherPos = launcherTranslation.plus(new Translation2d(
                    velocityForLookahead.vxMetersPerSecond * tof,
                    velocityForLookahead.vyMetersPerSecond * tof));
            lookaheadDist = target.getDistance(lookaheadLauncherPos);
        }

        // Drive angle from lookahead robot pose
        Pose2d lookaheadRobotPose = new Pose2d(lookaheadLauncherPos, launcherPose.getRotation())
                .transformBy(robotToLauncher.inverse());
        Rotation2d driveAngle = getDriveAngleWithLauncherOffset(lookaheadRobotPose, target);

        // Hood angle and shooter velocity
        double hoodAngleDeg = (passing
                ? passingHoodAngleMap.get(lookaheadDist)
                : hoodAngleMap.get(lookaheadDist)).getDegrees()
                + hoodAngleOffsetDeg;
        double shooterVelocity = passing
                ? passingShooterVelocityMap.get(lookaheadDist)
                : shooterVelocityMap.get(lookaheadDist);

        // Derivatives for feedforward
        if (lastDriveAngle == null) lastDriveAngle = driveAngle;
        if (Double.isNaN(lastHoodAngleDeg)) lastHoodAngleDeg = hoodAngleDeg;

        double hoodVelocity = hoodAngleFilter.calculate(
                (hoodAngleDeg - lastHoodAngleDeg) / loopPeriodSecs);
        double driveVelocity = driveAngleFilter.calculate(
                driveAngle.minus(lastDriveAngle).getRadians() / loopPeriodSecs);

        lastHoodAngleDeg = hoodAngleDeg;
        lastDriveAngle = driveAngle;

        // Validity
        boolean isValid = passing
                ? (lookaheadDist >= passingMinDistance && lookaheadDist <= passingMaxDistance)
                : (lookaheadDist >= minDistance && lookaheadDist <= maxDistance);

        latestParameters = new LaunchingParameters(
                isValid,
                driveAngle,
                driveVelocity,
                hoodAngleDeg,
                hoodVelocity,
                shooterVelocity,
                lookaheadDist,
                rawDistance,
                tof,
                passing);

        // Logging

        SmartDashboard.putData("Lookahead shoot on the move Field", m_field);
        m_field.setRobotPose(lookaheadRobotPose);
        //m_field.setRobotPose(estimatedPose);
        Logger.recordOutput("LaunchCalculator/Passing", passing);
        Logger.recordOutput("LaunchCalculator/TargetPose",
                new Pose2d(target, Rotation2d.fromDegrees(0)));
        Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadRobotPose);
        SmartDashboard.putNumber("LaunchCalculator/LookaheadDistance", lookaheadDist);
        SmartDashboard.putNumber("LaunchCalculator/RawDistance", rawDistance);
        SmartDashboard.putNumber("LaunchCalculator/DriveAngleDeg", driveAngle.getDegrees());
        SmartDashboard.putNumber("LaunchCalculator/HoodAngleDeg", hoodAngleDeg);
        SmartDashboard.putNumber("LaunchCalculator/ShooterVelocity", shooterVelocity);
        SmartDashboard.putNumber("LaunchCalculator/TimeOfFlight", tof);
         SmartDashboard.putBoolean("LaunchCalculator/IsValid", isValid);
        SmartDashboard.putNumber("LaunchCalculator/HoodAngleOffsetDeg", hoodAngleOffsetDeg);
       // Logger.recordOutput("LaunchCalculator/UsingSetpointVelocity", DriverStation.isAutonomous());
        SmartDashboard.putNumber("LaunchCalculator/VelocityForLookaheadX", velocityForLookahead.vxMetersPerSecond);
        SmartDashboard.putNumber("LaunchCalculator/VelocityForLookaheadY", velocityForLookahead.vyMetersPerSecond);

        return latestParameters;
    }

    // -------------------------------------------------------------------------
    // Stationary aimed pose
    // Returns the Pose2d the robot should face to aim at the hub (or pass target)
    // from a given translation, without any movement compensation.

    public static Pose2d getStationaryAimedPose(
            Translation2d robotTranslation,
            Translation2d hubTarget,
            boolean passing,
            Pose2d currentPose) {
        Translation2d target = passing ? getPassingTarget(currentPose) : hubTarget;
        return new Pose2d(
                robotTranslation,
                getDriveAngleWithLauncherOffset(new Pose2d(robotTranslation, new Rotation2d()), target));
    }

    // -------------------------------------------------------------------------
    // Passing target: fixed field point, mirrored vertically based on robot position
    // -------------------------------------------------------------------------
    public static Translation2d getPassingTarget(Pose2d robotPose) {
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        double fieldWidth = FieldConstants.fieldWidth;
        boolean mirror = robotPose.getY() > fieldWidth / 2.0;

        double x = isRed ? FieldConstants.fieldLength - xPassTarget : xPassTarget;
        double y = mirror ? fieldWidth - yPassTarget : yPassTarget;
        return new Translation2d(x, y);
}

    // -------------------------------------------------------------------------
    // Launcher velocity transform
    // If the robot is rotating and the launcher is offset from center,
    // the launcher tip moves at a different velocity than the robot center.
    // This adds that rotational contribution to the linear velocity.
    // -------------------------------------------------------------------------
    private static ChassisSpeeds transformVelocityToLauncher(
            ChassisSpeeds fieldVelocity,
            ChassisSpeeds robotVelocity,
            Rotation2d robotAngle) {
        // Offset from robot center to launcher in field frame
        double offsetX = robotToLauncher.getTranslation().getX();
        double offsetY = robotToLauncher.getTranslation().getY();

        // Rotate offset to field frame
        double cosA = robotAngle.getCos();
        double sinA = robotAngle.getSin();
        double fieldOffsetX = cosA * offsetX - sinA * offsetY;
        double fieldOffsetY = sinA * offsetX + cosA * offsetY;

        // Rotational velocity contribution: v = omega × r
        double omega = robotVelocity.omegaRadiansPerSecond;
        double extraVx = -omega * fieldOffsetY;
        double extraVy =  omega * fieldOffsetX;

        return new ChassisSpeeds(
                fieldVelocity.vxMetersPerSecond + extraVx,
                fieldVelocity.vyMetersPerSecond + extraVy,
                fieldVelocity.omegaRadiansPerSecond);
    }

    // -------------------------------------------------------------------------
    // Drive angle geometry
    // -------------------------------------------------------------------------
    private static Rotation2d getDriveAngleWithLauncherOffset(
            Pose2d robotPose, Translation2d target) {
        Rotation2d fieldToHubAngle = target.minus(robotPose.getTranslation()).getAngle();
        double lateralOffset = robotToLauncher.getTranslation().getY();
        double distToTarget = target.getDistance(robotPose.getTranslation());
        Rotation2d hubAngle = new Rotation2d(
                Math.asin(MathUtil.clamp(lateralOffset / distToTarget, -1.0, 1.0)));
        return fieldToHubAngle
                .plus(hubAngle)
                .plus(robotToLauncher.getRotation());
    }

    // -------------------------------------------------------------------------
    // Utilities
    // -------------------------------------------------------------------------
    private static double getTOF(boolean passing, double distance) {
        return passing
                ? passingTimeOfFlightMap.get(distance)
                : timeOfFlightMap.get(distance);
    }

    public void clearLaunchingParameters() {
        latestParameters = null;
    }

    public void incrementHoodAngleOffset(double incrementDegrees) {
        hoodAngleOffsetDeg += incrementDegrees;
    }

    public double getHoodAngleOffsetDeg() {
        return hoodAngleOffsetDeg;
    }

    public double getNaiveTOF(double distance) {
        return timeOfFlightMap.get(distance);
    }
    
    public static double getHoodAngleDeg(double distance) {
        return hoodAngleMap.get(distance).getDegrees();
    }

    public static double getShooterVelocity(double distance) {
        return shooterVelocityMap.get(distance);
    }

    public static double getPassingHoodAngleDeg(double distance) {
        return passingHoodAngleMap.get(distance).getDegrees();
    }

    public static double getPassingShooterVelocity(double distance) {
        return passingShooterVelocityMap.get(distance);
    }
}