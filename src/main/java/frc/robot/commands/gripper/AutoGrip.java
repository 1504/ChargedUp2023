// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Lidar;
import frc.robot.subsystems.Lidar.Action;

public class AutoGrip extends CommandBase {
  
  private Gripper m_grip = Gripper.getInstance();
  private Lidar m_lidar = Lidar.getInstance();
  private double input;

  public AutoGrip(double in) {
    input = in;
    addRequirements(m_grip);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(input);
    if(input > 0.1) {
      System.out.println("hi");
      if (m_lidar.getAction() == Action.GRIP) {
        new Open().schedule();
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
