package frc.robot.Autos;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class Autos {
    private final SendableChooser<Command> autoChooser;

    public Autos() {
        autoChooser = AutoBuilder.buildAutoChooser();
    }

    public SendableChooser<Command> getAutoChooser() {
        return autoChooser;
    }
}