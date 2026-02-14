package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.swerveConstants;
import frc.robot.Subsystems.Swerve.Swerve;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;

  public TeleopSwerve(
      Swerve s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
  }

  @Override
  public void execute() {
    /* Get Values, Deadband*/
    double translationVal = Math.pow(MathUtil.applyDeadband(translationSup.getAsDouble(), 0.05), 3);
    double strafeVal = Math.pow(MathUtil.applyDeadband(strafeSup.getAsDouble(), 0.05), 3);
    double rotationVal = Math.pow(MathUtil.applyDeadband(rotationSup.getAsDouble(), 0.05), 1);
    translationVal =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? translationVal
            : translationVal * -1;
    strafeVal =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? strafeVal
            : strafeVal * -1;
    rotationVal =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? rotationVal
            : rotationVal;

    double x_speed = translationVal * swerveConstants.moduleConstants.maxSpeedMeterPerSecond;
    double y_speed = strafeVal * swerveConstants.moduleConstants.maxSpeedMeterPerSecond;
    double rot_speed = rotationVal * swerveConstants.moduleConstants.maxAngularVelocity;

    /* Drive */
    s_Swerve.requestDesiredState(x_speed, y_speed, rot_speed, false, false);
  }
}
