// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.balance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gyroscope;
import frc.robot.commands.drive.Cartesian;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoBalance extends CommandBase {
  private static final Drivetrain drivetrain = Drivetrain.getInstance();
  private static final Gyroscope _gyro = Gyroscope.getInstance();
  private boolean finished = false;
  private int count = 0;

  /** Creates a new AutoBalance. */
  public AutoBalance() {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(drivetrain);
  }

  Timer timer;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * Balances the robot by adjusting the speed of the motors
   */
  @Override
  public void execute() {
    if (finished == true && Math.abs(_gyro.getRoll()) < 10) {
      drivetrain.cartesianDrive(0, 0, 0);
    } else {
      finished = false;
    }

    if (timer.get() > 1 && !finished) {
      System.out.println("Autobalancing    Pitch: " + _gyro.getPitch() + " Roll: " + _gyro.getRoll() + " Yaw: "
          + _gyro.getYaw());

      SmartDashboard.putString("Autobalance Status", "Autobalancing    Pitch: " + _gyro.getPitch() + " Roll: "
          + _gyro.getRoll() + " Yaw: " + _gyro.getYaw());

      // get the current pitch
      double roll = _gyro.getRoll();
      // if pitch is greater than 0, then the robot is leaning forward
      if (Math.abs(roll) < 10) { // if the pitch is close to zero, then stop the robot TODO: verify the
        // value of 0.1
        count++; // if the robot is balanced, increment the count, if not, reset the count
        if (count > 5) { // if the robot is balanced for 6 seconds, stop the robot
          // stop the robot
          // drivetrain.cartesianDrive(0, 0, 0);
          drivetrain.cartesianDrive(0, 0, 0);
          finished = true;
        }
      } else if (roll > 0) {
        // if the robot is leaning forward, move the robot backwards
        // drivetrain.cartesianDrive(-0.1, 0, 0); // TODO: add and verify delay to
        // prevent the robot from overshooting
        new Cartesian(() -> -0.1, () -> 0, () -> 0).schedule();
        count = 0;
      } else if (roll < 0) {
        // if the robot is leaning backward, move the robot forward
        // drivetrain.cartesianDrive(0.1, 0, 0);
        new Cartesian(() -> 0.1, () -> 0, () -> 0).schedule();
        count = 0;
      }

      timer.reset();
    } else {
      drivetrain.cartesianDrive(0, 0, 0);
    }
    SmartDashboard.putBoolean("Autobalanced", finished);
    if (finished) {
      SmartDashboard.putString("Autobalance Status", "Autobalanced");
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.out.println("Autobalance interrupted");
      finished = false;
      count = 0;
    }
    drivetrain.cartesianDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return finished;
    return false;
  }
}
