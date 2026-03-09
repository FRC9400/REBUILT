package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;


public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double appliedVolts = 0.0;
        public double shooterSetpointMPS = 0.0;
        public double appliedVelocity;
        public double[] shooterVelMPS = new double[] {};
        public double[] currentAmps = new double[] {};
        public double[] shooterVoltage = new double[] {};
        public double[] temp = new double[] {};

    }

    public default void updateInputs(ShooterIOInputs inputs) {
    }

    public default void setVelocity(double velocity) {
    }

    public default void zeroVelocity() {
    }

    public default void requestVoltage(double volts){
    }
}