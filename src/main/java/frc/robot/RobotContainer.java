// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Subsystems.Hood.Hood;
import frc.robot.Subsystems.Hood.HoodIOTalonFX;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeIOTalonFX;
import frc.robot.Subsystems.Rollers.Rollers;
import frc.robot.Subsystems.Rollers.RollersIOTalonFX;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterIOTalonFX;
import frc.robot.Subsystems.Swerve.Swerve;

public class RobotContainer {
  public static final CommandXboxController driver = new CommandXboxController(0);
  public static final CommandXboxController operator = new CommandXboxController(1);
  private final Swerve swerve = new Swerve();
  private final Shooter shooter = new Shooter(new ShooterIOTalonFX());
  private final Hood hood = new Hood(new HoodIOTalonFX());
  private final Intake intake = new Intake(new IntakeIOTalonFX());
  private final Rollers rollers = new Rollers(new RollersIOTalonFX());
  


  public RobotContainer() {
    swerve.zeroGyro();
    swerve.zeroWheels();
    swerve.setDefaultCommand(
        new TeleopSwerve(
            swerve,
            () -> -driver.getRawAxis(XboxController.Axis.kLeftY.value),
            () -> -driver.getRawAxis(XboxController.Axis.kLeftX.value),
            () -> -driver.getRawAxis(XboxController.Axis.kRightX.value)));

    configureBindings();
  }

  private void configureBindings() {
    //operator.a().onTrue(shooter.shooterSysIdCmd());
    driver.y().onTrue(new RunCommand(() -> swerve.zeroGyro()));
    driver.rightBumper().whileTrue(new RunCommand(() -> intake.requestIntakeVoltage(4)));
    driver.rightBumper().whileFalse(new RunCommand(() -> intake.requestIntakeVoltage(0)));
    driver.leftBumper().whileTrue(new RunCommand(() -> shooter.requestVoltage(7.2)));
    driver.leftBumper().whileFalse(new RunCommand(() -> shooter.requestVoltage(0)));
    driver.leftBumper().whileTrue(new RunCommand(() -> rollers.requestVoltage(4.5)));
    driver.a().whileTrue(new RunCommand(() -> rollers.requestVoltage(-2)));
    driver.b().whileTrue(new RunCommand(() -> rollers.requestVoltage(0)));
  }

  public Swerve getSwerve() {
    return swerve;
  }
}
