// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyroscope;

public class DriveDistanceForward extends CommandBase {
  
  Gyroscope gyro;
  Drivetrain drive;
  double distance;
  double curr_pos;
  double target;

  public DriveDistanceForward(double d) {
    gyro = Gyroscope.getInstance();
    drive = Drivetrain.getInstance();

    curr_pos = gyro.getDisplacementY();
    distance = d;
    target = curr_pos + distance;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.cartesianDrive(0.5, 0, 0);
    curr_pos = gyro.getDisplacementX();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(gyro.getDisplacementX() - curr_pos) < 0.1;
  }
}
