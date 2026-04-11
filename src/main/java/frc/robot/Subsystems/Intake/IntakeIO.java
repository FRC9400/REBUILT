package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double pivotAppliedVolts = 0;
        public double pivotSupplyCurrent = 0;
        public double pivotStatorCurrent = 0;
        public double pivotSetpointDeg = 0;
        public double pivotSetpointRot = 0;
        public double pivotPosDeg = 0;
        public double pivotPosRot = 0;
        public double pivotTemperature = 0;
        public double pivotVoltage = 0;
        public double pivotRPS = 0;

        public double intakeTemperature = 0;
        public double intakeAppliedVolts = 0;
        public double intakeCurrent = 0;
        public double intakeRPS = 0;
        public double intakeVoltage = 0;
        public double intakeSetpointVolts = 0;

        public double intake2Temperature = 0;
        public double intake2Current = 0;
        public double intake2RPS = 0;
        public double intake2Voltage = 0;
    }

    public default void updateInputs (IntakeIOInputs inputs) {}

    public default void requestPivotVoltage (double voltage) {}

    public default void requestSetpoint (double angleDegrees) {}
    
    public default void requestIntakeVoltage(double voltage) {}

    public default void zeroPosition() {}

}