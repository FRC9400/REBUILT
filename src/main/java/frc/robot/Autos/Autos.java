package frc.robot.Autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Swerve.Swerve;

public class Autos {
    private final SendableChooser<Command> autoChooser;

    public Autos(Swerve swerve, Superstructure superstructure) {
        ExampleAuto exampleAuto = new ExampleAuto(swerve, superstructure);
        MadelineAuto madelineAuto = new MadelineAuto(swerve, superstructure);

        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("Example Auto", exampleAuto.getCommand());
         autoChooser.addOption("Madeline Auto", madelineAuto.getCommand());
    }

    public SendableChooser<Command> getAutoChooser() {
        return autoChooser;
    }
}