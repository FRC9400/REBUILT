package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeInputs{
        public double appliedVolts = 0;
        public double setpointVolts = 0;
        public double voltage = 0;
        public double velocityRPS = 0;
        public double currentAmps = 0;
        public double tempFahrenheit = 0;
    }

    public default void updateInputs(IntakeInputs inputs){}
    
    public default void requestVoltage(double volts){}
}
