// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;

public class Open extends CommandBase {

  private static final Gripper m_gripper = Gripper.getInstance();

  /** Creates a new Open. */
  public Open() {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gripper.open();
  }

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * Opens the gripper
   */

  @Override
  public void execute() {
    m_gripper.open();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_gripper.open();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
