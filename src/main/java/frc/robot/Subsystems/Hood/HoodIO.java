package frc.robot.Subsystems.Hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    public class HoodIOInputs{
        public double setpointDeg = 0;
        public double setpointVolts = 0;
        public double setpointRot;

        public double appliedDeg = 0;
        public double appliedVolts = 0;

        public double current = 0;
        public double temp = 0;
        public double rps = 0;
    }

    public default void updateInputs(HoodIOInputs hoodIOInputs){}
    
    public default void requestMotionMagic(double setpointDeg){}

    public default void requestVoltage(double setpointVolts){}
    
}