package frc.commons;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmPosition {
  private double extensionLengthMeters = 0;
  private Rotation2d shoulderAngleRot2d = new Rotation2d();
  private Rotation2d wristAngleRot2d = new Rotation2d();

  public ArmPosition(
      double extensionLengthMeters, Rotation2d shoulderAngleRot2d, Rotation2d wristAngleRot2d) {
    this.extensionLengthMeters = extensionLengthMeters;
    this.shoulderAngleRot2d = shoulderAngleRot2d;
    this.wristAngleRot2d = wristAngleRot2d;
  }

  public Rotation2d getShoulderAngleRot2d() {
    return shoulderAngleRot2d;
  }

  public Rotation2d getWristAngleRot2d() {
    return wristAngleRot2d;
  }

  public double getExtensionLengthMeters() {
    return extensionLengthMeters;
  }
}
