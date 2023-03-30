// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class GoToAngle extends CommandBase {
  
  private Drivetrain m_drive = Drivetrain.getInstance();
  private double targetAngle;

  public GoToAngle(double angle) {
    targetAngle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.enableRotate();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.rotatePID();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.disableRotate();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drive.isAngled(targetAngle);
  }
}
