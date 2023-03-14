// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.balance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gyroscope;
import frc.robot.subsystems.Drivetrain;

// get the current time in seconds
import edu.wpi.first.wpilibj.Timer;

public class AutoBalance extends CommandBase {
  Drivetrain drivetrain;
  private static Gyroscope _gyro;

  /** Creates a new AutoBalance. */
  public AutoBalance() {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = new Drivetrain(); // verify that this is the correct way to get the drivetrain
    _gyro = Gyroscope.getInstance();
    addRequirements(drivetrain);
  }

  Timer timer;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    timer.start();

  }

  int count = 0;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (timer.get() > 1) {

      // get the current pitch
      double pitch = _gyro.getPitch();
      // if pitch is greater than 0, then the robot is leaning forward
      if (Math.pow(pitch, 2) < 0.1) { // if the pitch is close to zero, then stop the robot TODO: verify the
        // value of 0.1
        count++; // if the robot is balanced, increment the count, if not, reset the count
        if (count > 5) { // if the robot is balanced for 6 seconds, stop the robot
          // stop the robot
          drivetrain.cartesianDrive(0, 0, 0);
        }
      } else if (pitch > 0) {
        // if the robot is leaning forward, move the robot backwards
        drivetrain.cartesianDrive(0, -0.11, 0); // TODO: add and verify delay to prevent the robot from overshooting
        count = 0;
      } else if (pitch < 0) {
        // if the robot is leaning backward, move the robot forward
        drivetrain.cartesianDrive(0, 0.11, 0);
        count = 0;
      }

      timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  // TODO: Verify if it's fine to leave this as false
  @Override
  public boolean isFinished() {
    return false;
  }
}
