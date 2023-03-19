// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class FollowTrajectory extends CommandBase {

  private static final Drivetrain m_drivetrain = Drivetrain.getInstance(); 

  /** Creates a new FollowTrajectory. */
  public FollowTrajectory() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  //@Override
  //TODO: i had to comment out override for the code to work which is giving me bad vibes and i dont think its right
  public void execute(PathPlannerTrajectory traj) {
    m_drivetrain.followTrajectory(traj);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
