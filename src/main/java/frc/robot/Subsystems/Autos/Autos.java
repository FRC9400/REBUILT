package frc.robot.Subsystems.Autos;


import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Swerve.Swerve;

public class Autos {
    private final Swerve swerve;
    private final AutoFactory autoFactory;

    public Autos(Swerve swerve){
        this.swerve = swerve;
        autoFactory = new AutoFactory(
                swerve::getPoseRaw,
                swerve::resetPose,
                swerve::followChoreoTraj,
                true,
                swerve);    
    }
    public AutoFactory getFactory() {
            return autoFactory;
    }

    public Command tune(){
        final AutoRoutine routine = autoFactory.newRoutine("Tune");
        final AutoTrajectory trajectory = routine.trajectory("NewPath");
        routine.active().whileTrue(Commands.sequence(
            trajectory.resetOdometry(),
            trajectory.cmd()
            )
        );
        return routine.cmd();
    }

    public Command none(){
        return new InstantCommand();
    }

}