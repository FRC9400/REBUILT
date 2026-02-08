package frc.robot.Subsystems.Rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {
    @AutoLog
    public static class RollersIOInputs{
        public double setpointVolts = 0;
        public double appliedVolts = 0;
        
        public double rollerCurrent = 0;
        public double rollerTemp = 0;
        public double rollerRPS = 0;
    }

    public default void updateInputs(RollersIOInputs rollersIOinputs){}
    public default void requestVoltage(double setpointVolts){}

}