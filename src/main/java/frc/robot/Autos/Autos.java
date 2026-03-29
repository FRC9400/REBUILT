package frc.robot.Autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;

public class Autos {
    private final SendableChooser<Command> autoChooser;

    public Autos() {
        autoChooser = AutoBuilder.buildAutoChooser(); 
    }

    public SendableChooser<Command> getAutoChooser() {
        return autoChooser;
    }

    public Command getSelectedCommand() {
        Command selected = autoChooser.getSelected();
        return selected != null ? selected : edu.wpi.first.wpilibj2.command.Commands.none();
    }

    public Optional<Pose2d> getSelectedStartingPose() {
        Command selected = autoChooser.getSelected();
        if (selected instanceof PathPlannerAuto auto) {
            return Optional.ofNullable(auto.getStartingPose());
        }
        return Optional.empty();
    }
}