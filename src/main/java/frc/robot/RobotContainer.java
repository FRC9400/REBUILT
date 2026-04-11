package frc.robot;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Autos.Autos;
import frc.robot.Commands.AutoShootCommand;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.ShootOnMoveCommand;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Constants.fieldConstants;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Hood.HoodIOTalonFX;
import frc.robot.Subsystems.Intake.IntakeIOTalonFX;
import frc.robot.Subsystems.Rollers.RollersIOTalonFX;
import frc.robot.Subsystems.Shooter.ShooterIOTalonFX;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.SnapToHubCommand;
import frc.robot.Subsystems.Swerve.Swerve;

public class RobotContainer {
  public static final CommandXboxController driver = new CommandXboxController(0);
  public static final CommandXboxController operator = new CommandXboxController(1);
  private final Swerve swerve = new Swerve();

private final Superstructure superstructure = new Superstructure(
    new HoodIOTalonFX(), new IntakeIOTalonFX(), new RollersIOTalonFX(), new ShooterIOTalonFX(),
    swerve::getDistanceToHub, swerve::getDistanceForPass);
  private final Autos autos;

  public RobotContainer() {
    superstructure.requestIdle();
    swerve.zeroGyro();
    swerve.zeroWheels();
    swerve.setDefaultCommand(
        new TeleopSwerve(
            swerve,
            () -> driver.getRawAxis(XboxController.Axis.kLeftY.value),
            () -> driver.getRawAxis(XboxController.Axis.kLeftX.value),
            () -> -driver.getRawAxis(XboxController.Axis.kRightX.value)));

    NamedCommands.registerCommand("Intake", new IntakeCommand(superstructure));
    NamedCommands.registerCommand("AutoShoot", new AutoShootCommand(superstructure));
    NamedCommands.registerCommand("Auto Align", new SnapToHubCommand(swerve));
    NamedCommands.registerCommand("ShootOnMove", new ShootOnMoveCommand(swerve,superstructure,false));
    NamedCommands.registerCommand("Idle", new InstantCommand(() -> superstructure.requestIdle()));
    NamedCommands.registerCommand("TogglePreSpin", Commands.runOnce(() -> superstructure.setPreSpin(!superstructure.isPreSpinEnabled())));

    autos = new Autos();
    SmartDashboard.putData("Auto Chooser", autos.getAutoChooser());

    configureBindings();
  }

  private void configureBindings() {    
    driver.x().onTrue(new InstantCommand(() -> superstructure.requestBump()));
    driver.y().onTrue(new InstantCommand(() -> swerve.zeroGyro()));
    driver.a().onTrue(new InstantCommand(() -> superstructure.requestUnJam()));
    driver.b().onTrue(new InstantCommand(() -> superstructure.requestIdle()));

    //driver.pov(0).onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));
    driver.start().onTrue(Commands.runOnce(() -> superstructure.setPreSpin(!superstructure.isPreSpinEnabled())));
    operator.x().onTrue(Commands.runOnce(() -> superstructure.setPreSpin(!superstructure.isPreSpinEnabled())));

    operator.rightBumper().onTrue(Commands.runOnce(() -> superstructure.togglePivotOverride()));
    driver.rightTrigger().whileTrue(new ShootOnMoveCommand(swerve, superstructure, true));
    
    driver.back().whileTrue(new InstantCommand(() -> superstructure.requestSpinUpToShoot()));
    driver.leftTrigger().whileTrue(new ShootOnMoveCommand(swerve, superstructure, false));
    
    //driver.rightTrigger().onTrue(new InstantCommand(() -> superstructure.requestSpinUpToShoot()));
    
    driver.leftBumper().onTrue(new InstantCommand(() -> superstructure.requestIntake()));
    driver.rightBumper().onTrue(new InstantCommand(() -> superstructure.requestSPITOUT()));
  }

  public Swerve getSwerve() {
    return swerve;
  }

  public Command getAutonomousCommand() {
    return autos.getAutoChooser().getSelected();
  }

  public Autos getAutos(){
    return autos;
  }

  public void makeIdle(){
    superstructure.requestIdle();
  }
}