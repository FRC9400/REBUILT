package frc.robot.Autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Swerve.Swerve;

public class ExampleAuto {
    private final Swerve swerve;
    private final Superstructure superstructure;

    public ExampleAuto(Swerve swerve, Superstructure superstructure) {
        this.swerve = swerve;
        this.superstructure = superstructure;
    }

    public Command getCommand() {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");
            return Commands.sequence(
                Commands.print("Starting ExampleAuto"),
                AutoBuilder.followPath(path)
            );
        } catch (Exception e) {  // parentheses, not curly braces
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }
}