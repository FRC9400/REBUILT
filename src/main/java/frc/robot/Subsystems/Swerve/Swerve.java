package frc.robot.Subsystems.Swerve;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.swerveConstants;
import frc.robot.Constants.swerveConstants.kinematicsConstants;
import frc.robot.LimelightHelpers;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {

  private final PIDController xController = new PIDController(4.6, 0, 0.023);
  private final PIDController yController = new PIDController(4.205, 0, 0.015);
  private final PIDController thetaController = new PIDController(3.57, 0, 0.005);

  private boolean feedLeft = false;
  private final GyroIO gyroIO = new GyroIOPigeon2(canIDConstants.pigeon);
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  public final ModuleIO[] moduleIOs = new ModuleIO[4];
  private final ModuleIOInputsAutoLogged[] moduleInputs = {
    new ModuleIOInputsAutoLogged(),
    new ModuleIOInputsAutoLogged(),
    new ModuleIOInputsAutoLogged(),
    new ModuleIOInputsAutoLogged()
  };
  private Pose2d poseRaw = new Pose2d();
  private Rotation2d lastGyroYaw = new Rotation2d();
  private final boolean fieldRelatve;
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          kinematicsConstants.FL,
          kinematicsConstants.FR,
          kinematicsConstants.BL,
          kinematicsConstants.BR);
  SwerveModuleState setpointModuleStates[] =
      kinematics.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, new Rotation2d()));

  private double[] lastModulePositionsMeters = new double[] {0.0, 0.0, 0.0, 0.0};
  SwerveDrivePoseEstimator poseEstimator; // add actual std devs later
  private final SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(kinematics, getRotation2d(), getSwerveModulePositions());

  private final SysIdRoutine driveRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              Seconds.of(1.5),
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism((Voltage volts) -> driveVoltage(volts.in(Volts)), null, this));

  private final SysIdRoutine steerRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(5),
              Seconds.of(6),
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (Voltage volts) -> moduleIOs[0].steerVoltage(volts.in(Volts)), null, this));

  public Swerve() {

    moduleIOs[0] =
        new ModuleIOTalonFX(
            canIDConstants.driveMotor[0],
            canIDConstants.steerMotor[0],
            canIDConstants.CANcoder[0],
            swerveConstants.moduleConstants.CANcoderOffsets[0],
            swerveConstants.moduleConstants.driveMotorInverts[0],
            swerveConstants.moduleConstants.steerMotorInverts[0],
            swerveConstants.moduleConstants.CANcoderInverts[0]);

    moduleIOs[1] =
        new ModuleIOTalonFX(
            canIDConstants.driveMotor[1],
            canIDConstants.steerMotor[1],
            canIDConstants.CANcoder[1],
            swerveConstants.moduleConstants.CANcoderOffsets[1],
            swerveConstants.moduleConstants.driveMotorInverts[1],
            swerveConstants.moduleConstants.steerMotorInverts[1],
            swerveConstants.moduleConstants.CANcoderInverts[1]);

    moduleIOs[2] =
        new ModuleIOTalonFX(
            canIDConstants.driveMotor[2],
            canIDConstants.steerMotor[2],
            canIDConstants.CANcoder[2],
            swerveConstants.moduleConstants.CANcoderOffsets[2],
            swerveConstants.moduleConstants.driveMotorInverts[2],
            swerveConstants.moduleConstants.steerMotorInverts[2],
            swerveConstants.moduleConstants.CANcoderInverts[2]);

    moduleIOs[3] =
        new ModuleIOTalonFX(
            canIDConstants.driveMotor[3],
            canIDConstants.steerMotor[3],
            canIDConstants.CANcoder[3],
            swerveConstants.moduleConstants.CANcoderOffsets[3],
            swerveConstants.moduleConstants.driveMotorInverts[3],
            swerveConstants.moduleConstants.steerMotorInverts[3],
            swerveConstants.moduleConstants.CANcoderInverts[3]);

    for (int i = 0; i < 4; i++) {
      moduleIOs[i].setDriveBrakeMode(true);
      moduleIOs[i].setTurnBrakeMode(false);
    }
    this.fieldRelatve = true;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    poseEstimator =
        new SwerveDrivePoseEstimator(kinematics, lastGyroYaw, getSwerveModulePositions(), poseRaw);
  }

  @Override
  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Swerve/Gyro", gyroInputs);
    for (int i = 0; i < 4; i++) {
      moduleIOs[i].updateInputs(moduleInputs[i]);
      Logger.processInputs("Swerve/Module/ModuleNum[" + i + "]", moduleInputs[i]);
    }

    updateOdometry();
    logModuleStates("SwerveModuleStates/setpointStates", getSetpointStates());
    // logModuleStates("SwerveModuleStates/optimizedSetpointStates", getOptimizedSetPointStates());
    logModuleStates("SwerveModuleStates/MeasuredStates", getMeasuredStates());
    Logger.recordOutput("Odometry/EstimatedPose", poseRaw);
    Logger.recordOutput("Odometry/PoseRaw", odometry.getPoseMeters());
    Logger.recordOutput("FeedLeft", feedLeft);
  }

  public void requestDesiredState(
      double x_speed, double y_speed, double rot_speed, boolean fieldRelative, boolean isOpenLoop) {

    Rotation2d[] steerPositions = new Rotation2d[4];
    SwerveModuleState[] desiredModuleStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      steerPositions[i] = new Rotation2d(moduleInputs[i].moduleAngleRads);
    }
    Rotation2d gyroPosition = new Rotation2d(gyroInputs.positionRad);
    if (fieldRelative && isOpenLoop) {
      desiredModuleStates =
          kinematics.toSwerveModuleStates(
              ChassisSpeeds.fromFieldRelativeSpeeds(x_speed, y_speed, rot_speed, gyroPosition));
      kinematics.desaturateWheelSpeeds(setpointModuleStates, 12);
      for (int i = 0; i < 4; i++) {
        setpointModuleStates[i] =
            SwerveModuleState.optimize(desiredModuleStates[i], steerPositions[i]);
        moduleIOs[i].setDesiredState(setpointModuleStates[i], true);
      }
    } else if (fieldRelative && !isOpenLoop) {
      desiredModuleStates =
          kinematics.toSwerveModuleStates(
              ChassisSpeeds.fromFieldRelativeSpeeds(x_speed, y_speed, rot_speed, gyroPosition));
      kinematics.desaturateWheelSpeeds(
          setpointModuleStates, swerveConstants.moduleConstants.maxSpeedMeterPerSecond);
      for (int i = 0; i < 4; i++) {
        setpointModuleStates[i] =
            SwerveModuleState.optimize(desiredModuleStates[i], steerPositions[i]);
        moduleIOs[i].setDesiredState(setpointModuleStates[i], false);
      }
    } else if (!fieldRelative && !isOpenLoop) {
      desiredModuleStates =
          kinematics.toSwerveModuleStates(new ChassisSpeeds(x_speed, y_speed, rot_speed));
      kinematics.desaturateWheelSpeeds(
          setpointModuleStates, swerveConstants.moduleConstants.maxSpeedMeterPerSecond);
      for (int i = 0; i < 4; i++) {
        setpointModuleStates[i] =
            SwerveModuleState.optimize(desiredModuleStates[i], steerPositions[i]);
        moduleIOs[i].setDesiredState(setpointModuleStates[i], false);
      }
    }
  }

  public void zeroWheels() {
    for (int i = 0; i < 4; i++) {
      moduleIOs[i].resetToAbsolute();
    }
  }

  public void setFeed(boolean feedLeft) {
    this.feedLeft = feedLeft;
  }

  public boolean getFeed() {
    return feedLeft;
  }

  public void zeroGyro() {
    gyroIO.reset();
  }

  public void updateOdometry() {
    var gyroYaw = new Rotation2d(gyroInputs.positionRad);
    lastGyroYaw = gyroYaw;
    poseEstimator.update(lastGyroYaw, getSwerveModulePositions());

    LimelightHelpers.SetRobotOrientation(
        "Weasel",
        poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);
    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    if (mt2.tagCount >= 1) {
      poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(1.7, 1.7, 20));
      poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
    }

    /*LimelightHelpers.SetRobotOrientation(
        "limelight",
        poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);
    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    if (mt1.tagCount != 0) {
      poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 10));
      poseEstimator.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
    }*/

    poseRaw = poseEstimator.getEstimatedPosition();
    odometry.update(getRotation2d(), getSwerveModulePositions());
  }

  public Pose2d getPoseRaw() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    setGyroStartingPosition(pose.getRotation().getDegrees());
    poseEstimator.resetPosition(getRotation2d(), getSwerveModulePositions(), pose);
    poseRaw = pose;
    odometry.resetPosition(getRotation2d(), getSwerveModulePositions(), pose);
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds desiredChassisSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    double x_speed = desiredChassisSpeeds.vxMetersPerSecond;
    double y_speed = desiredChassisSpeeds.vyMetersPerSecond;
    double rot_speed = desiredChassisSpeeds.omegaRadiansPerSecond;

    requestDesiredState(x_speed, y_speed, rot_speed, false, false);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getMeasuredStates());
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(gyroInputs.positionRad);
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      modulePositions[i] =
          new SwerveModulePosition(
              moduleInputs[i].driveDistanceMeters, new Rotation2d(moduleInputs[i].moduleAngleRads));
    }
    return modulePositions;
  }

  public SwerveModuleState[] getMeasuredStates() {
    SwerveModuleState[] measuredStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      measuredStates[i] =
          new SwerveModuleState(
              moduleInputs[i].driveVelocityMetersPerSec,
              new Rotation2d(moduleInputs[i].moduleAngleRads));
    }
    return measuredStates;
  }

  public void driveVoltage(double volts) {
    for (int i = 0; i < 4; i++) {
      moduleIOs[i].setDriveVoltage(volts);
    }
  }

  public Command driveSysIdCmd() {
    return Commands.sequence(
        this.runOnce(() -> SignalLogger.start()),
        driveRoutine.quasistatic(Direction.kForward),
        this.runOnce(() -> driveVoltage(0)),
        Commands.waitSeconds(1),
        driveRoutine.quasistatic(Direction.kReverse),
        this.runOnce(() -> driveVoltage(0)),
        Commands.waitSeconds(1),
        driveRoutine.dynamic(Direction.kForward),
        this.runOnce(() -> driveVoltage(0)),
        Commands.waitSeconds(1),
        driveRoutine.dynamic(Direction.kReverse),
        this.runOnce(() -> driveVoltage(0)),
        Commands.waitSeconds(1),
        this.runOnce(() -> SignalLogger.stop()));
  }

  public Command steerSysIdCmd() {
    return Commands.sequence(
        this.runOnce(() -> SignalLogger.start()),
        steerRoutine.quasistatic(Direction.kForward),
        this.runOnce(() -> moduleIOs[0].steerVoltage(0)),
        Commands.waitSeconds(1),
        steerRoutine.quasistatic(Direction.kReverse),
        this.runOnce(() -> moduleIOs[0].steerVoltage(0)),
        Commands.waitSeconds(1),
        steerRoutine.dynamic(Direction.kForward),
        this.runOnce(() -> moduleIOs[0].steerVoltage(0)),
        Commands.waitSeconds(1),
        steerRoutine.dynamic(Direction.kReverse),
        this.runOnce(() -> moduleIOs[0].steerVoltage(0)),
        Commands.waitSeconds(1),
        this.runOnce(() -> SignalLogger.stop()));
  }

  public SwerveModuleState[] getSetpointStates() {
    return setpointModuleStates;
  }

  public double getGyroPositionDegrees() {
    return gyroInputs.positionDegRaw;
  }

  public double getGyroPositionRadians() {
    return gyroInputs.positionRad;
  }

  public double getDriveCurrent() {
    return moduleInputs[0].driveCurrentAmps;
  }

  public void setGyroStartingPosition(double yawDegrees) {
    gyroIO.setPosition(yawDegrees);
  }

  private void logModuleStates(String key, SwerveModuleState[] states) {
    List<Double> dataArray = new ArrayList<Double>();
    for (int i = 0; i < 4; i++) {
      dataArray.add(states[i].angle.getRadians());
      dataArray.add(states[i].speedMetersPerSecond);
    }
    Logger.recordOutput(key, dataArray.stream().mapToDouble(Double::doubleValue).toArray());
  }

  public void followChoreoTraj(SwerveSample sample) {
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(
                sample.vx
                    + xController.calculate(poseEstimator.getEstimatedPosition().getX(), sample.x),
                sample.vy
                    + yController.calculate(poseEstimator.getEstimatedPosition().getY(), sample.y),
                sample.omega
                    + thetaController.calculate(
                        poseEstimator.getEstimatedPosition().getRotation().getRadians(),
                        sample.heading)),
            poseEstimator.getEstimatedPosition().getRotation());
    driveRobotRelative(speeds);
  }
}
