// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;

public class Close extends CommandBase {

  private static final Gripper m_gripper = Gripper.getInstance();

  public Close() {

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gripper.close();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    m_gripper.close();
    System.out.println("me when I close");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_gripper.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
