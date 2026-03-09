package frc.robot.Constants;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class fieldConstants {

    public static final Pose2d BLUE_HUB_POS =
      new Pose2d(4.71678, 4.13004, Rotation2d.kZero);
  public static final Pose2d RED_HUB_POS = ChoreoAllianceFlipUtil.flip(BLUE_HUB_POS);
    
}
