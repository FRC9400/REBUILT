package frc.robot.Subsystems.Hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    public class HoodIOInputs{
        public double hoodSetpointVolts = 0;
        public double hoodSetpointDeg = 0;
        public double hoodSetpointRot = 0;
        public double hoodAppliedVolts = 0;
        public double hoodAppliedDeg = 0;
        public double hoodCurrent = 0;
        public double hoodRPS = 0;
        public double hoodTemp = 0;
        public double hoodPosDeg = 0;
        public double hoodPosRot = 0;
    }

    public default void updateInputs(HoodIOInputs hoodIOInputs){}
    
    public default void requestMotionMagic(double setpointDeg){}

    public default void requestVoltage(double setpointVolts){}
    
}