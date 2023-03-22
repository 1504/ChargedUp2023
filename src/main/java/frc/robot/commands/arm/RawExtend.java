// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class RawExtend extends CommandBase {
  
  private static final Arm m_arm = Arm.getInstance();

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * Extends the arm using the raw motor
   */
  @Override
  public void execute() {
    m_arm.setAuto(false);
    m_arm.rawExtend();
    //m_arm.PIDDrive(ArmConstants.kExtendSetpoint);
    System.out.println("me when I extend");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
