// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;


import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import swervelib.SwerveInputStream;

public class RobotContainer {

  private CommandXboxController m_driveController = new CommandXboxController(0);
  private Drivetrain swerve = new Drivetrain(new File(Filesystem.getDeployDirectory(), "Swerve"));
  public RobotContainer() {
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerve.getSwerveDrive(), ()-> m_driveController.getLeftY() * -1, ()->m_driveController.getLeftX() * -1).withControllerRotationAxis(m_driveController::getRightX).deadband(.3).scaleTranslation(0.8).allianceRelativeControl(true);
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driveController::getRightX, m_driveController::getRightY).headingWhile(true);
    configureBindings();
    swerve.setDefaultCommand(swerve.drive(
                            ()->m_driveController.getLeftX(),
                            ()->m_driveController.getLeftY(),   
                            ()->m_driveController.getRightX(),  
                            true));

    
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
