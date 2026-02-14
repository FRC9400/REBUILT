// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeIOTalonFX;
import frc.robot.Subsystems.Swerve.Swerve;

public class RobotContainer {
  public static final CommandXboxController driver = new CommandXboxController(0);
  public static final CommandXboxController operator = new CommandXboxController(1);
  private final Swerve swerve = new Swerve();
  private final Intake intake = new Intake(new IntakeIOTalonFX());

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
    operator.a().whileTrue(new RunCommand(() -> intake.requestVoltage(3)));
    operator.b().whileTrue(new RunCommand(() -> intake.requestVoltage(0)));
    operator.x().whileTrue(new RunCommand(() -> intake.requestVoltage(-3)));

    driver.y().whileTrue(new RunCommand(() -> swerve.zeroGyro()));
  }

  public Swerve getSwerve() {
    return swerve;
  }
}
