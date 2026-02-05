package frc.robot.Subsystems.Rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {
    @AutoLog
    public static class RollersIOInputs{
        public double setpointVolts = 0;
        public double appliedVolts = 0;
        
        public double leftRollerCurrent = 0;
        public double rightRollerCurrent = 0;
        public double leftRollerTemp = 0;
        public double rightRollerTemp = 0;
        public double leftRollerRPS = 0;
        public double rightRollerRPS = 0;
    }

    public default void updateInputs(RollersIOInputs rollersIOinputs){}
    public default void requestVoltage(double setpointVolts){}

}