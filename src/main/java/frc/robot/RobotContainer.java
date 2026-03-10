// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Autos.Autos;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Hood.Hood;
import frc.robot.Subsystems.Hood.HoodIO;
import frc.robot.Subsystems.Hood.HoodIOTalonFX;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeIO;
import frc.robot.Subsystems.Intake.IntakeIOTalonFX;
import frc.robot.Subsystems.Rollers.Rollers;
import frc.robot.Subsystems.Rollers.RollersIO;
import frc.robot.Subsystems.Rollers.RollersIOTalonFX;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterIO;
import frc.robot.Subsystems.Shooter.ShooterIOTalonFX;
import frc.robot.Subsystems.Swerve.SnapToHubCommand;
import frc.robot.Subsystems.Swerve.Swerve;

public class RobotContainer {
  public static final CommandXboxController driver = new CommandXboxController(0);
  public static final CommandXboxController operator = new CommandXboxController(1);
  private final Swerve swerve = new Swerve();
  private final HoodIO s_hood = new HoodIOTalonFX();
  private final IntakeIO s_intake = new IntakeIOTalonFX();
  private final RollersIO s_rollers = new RollersIOTalonFX();
  private final ShooterIO s_shooter = new ShooterIOTalonFX();
  private final Superstructure superstructure = new Superstructure(s_hood, s_intake, s_rollers, s_shooter, swerve::getDistanceToHub);
  private final Autos autos = new Autos(swerve, superstructure);

  public RobotContainer() {
    swerve.zeroGyro();
    swerve.zeroWheels();
    swerve.setDefaultCommand(
        new TeleopSwerve(
            swerve,
            () -> driver.getRawAxis(XboxController.Axis.kLeftY.value),
            () -> driver.getRawAxis(XboxController.Axis.kLeftX.value),
            () -> -driver.getRawAxis(XboxController.Axis.kRightX.value)));
    SmartDashboard.putData("Auto Chooser", autos.getAutoChooser());
    configureBindings();
  }

  private void configureBindings() {
    driver.y().onTrue(new InstantCommand(() -> swerve.zeroGyro()));
    driver.a().onTrue(new InstantCommand(() -> superstructure.requestIdle()));
    driver.b().onTrue(new InstantCommand(() -> superstructure.requestBump()));
    driver.x().onTrue(new InstantCommand(() -> superstructure.requestIntake()));
    driver.leftBumper().onTrue(new InstantCommand(() -> superstructure.requestOuttake()));
    driver.rightBumper().onTrue(new InstantCommand(() -> superstructure.requestUnJam()));
    driver.leftTrigger().whileTrue(new SnapToHubCommand(swerve));
    driver.rightTrigger().onTrue(new InstantCommand(() -> superstructure.requestAUTOSpinUp()));
  }

  public Swerve getSwerve() {
    return swerve;
  }

  public Command getAutonomousCommand() {
    return autos.getAutoChooser().getSelected();
}
}
