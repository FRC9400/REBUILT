package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

public class GyroIOPigeon2 implements GyroIO {
  Pigeon2 pigeon;
  private final StatusSignal<Angle> positionDegRaw;
  private final StatusSignal<Angle> pitchDeg;
  private final StatusSignal<Angle> rollDeg;

  public GyroIOPigeon2(int pigeonID) {
    pigeon = new Pigeon2(pigeonID, "canivore");
    positionDegRaw = pigeon.getYaw();
    pitchDeg = pigeon.getPitch();
    rollDeg = pigeon.getRoll();

    BaseStatusSignal.setUpdateFrequencyForAll(250, positionDegRaw, pitchDeg, rollDeg);

    pigeon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    BaseStatusSignal.refreshAll(positionDegRaw, pitchDeg, rollDeg);
    inputs.connected = true;
    inputs.positionDegRaw = positionDegRaw.getValueAsDouble();
    inputs.positionRad = Units.degreesToRadians(positionDegRaw.getValueAsDouble());
    inputs.positionDegRawAbsolute = positionDegRaw.getValueAsDouble() % 360;
    inputs.velocityRadPerSec = 0.0;
    inputs.pitchDeg = pitchDeg.getValueAsDouble();
    inputs.rollDeg = rollDeg.getValueAsDouble();
    inputs.pitchRad = Units.degreesToRadians(pitchDeg.getValueAsDouble());
    inputs.rollRad = Units.degreesToRadians(rollDeg.getValueAsDouble());
  }

  @Override
  public void reset() {
    pigeon.setYaw(0.0);
  }

  public void setPosition(double yawDegrees) {
    pigeon.setYaw(yawDegrees);
  }
}
