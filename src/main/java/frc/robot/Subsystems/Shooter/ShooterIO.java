package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;


public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double appliedVolts = 0.0;
        public double[] shooterSetpointsMPS = new double[] {};
        public double[] shooterVelMPS = new double[] {};
        public double[] currentAmps = new double[] {};
        public double[] tempF = new double[] {};

    }

    public default void updateInputs(ShooterIOInputs inputs) {
    }

    public default void setVelocity(double velocity, double ratio) {
    }

    public default void zeroVelocity() {
    }
}
