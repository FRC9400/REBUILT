package frc.robot.Autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Commands.AutoShootCommand;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Swerve.Swerve;

public class MadelineAuto {
    private final Swerve swerve;
    private final Superstructure superstructure;

    public MadelineAuto(Swerve swerve, Superstructure superstructure) {
        this.swerve = swerve;
        this.superstructure = superstructure;
    }

    public Command getCommand() {
        try {
            PathPlannerPath path1 = PathPlannerPath.fromPathFile("Blue.Mid-IntakeBoxA");
            PathPlannerPath path2 = PathPlannerPath.fromPathFile("Blue.IntakeBoxA-B");
            PathPlannerPath path3 = PathPlannerPath.fromPathFile("Blue.IntakeBoxB-Mid");
            return Commands.sequence(
              //  new AutoShootCommand(superstructure),
                AutoBuilder.followPath(path1),
                new ParallelDeadlineGroup(
                    AutoBuilder.followPath(path2),
                    new IntakeCommand(superstructure)
                ),
                AutoBuilder.followPath(path3)
            );
        } catch (Exception e) {  // parentheses, not curly braces
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }
}