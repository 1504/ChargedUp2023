// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.arm.RawExtend;
import frc.robot.commands.arm.SetArmPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

public class AutonExtend extends CommandBase {
  
  private Drivetrain m_drive = Drivetrain.getInstance();
  private Arm m_arm = Arm.getInstance();

  public AutonExtend() {
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new SetArmPosition(50).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new SetArmPosition(0).schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
