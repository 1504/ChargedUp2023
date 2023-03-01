// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
   /* Motor Controllers */
   private final CANSparkMax _front_left_motor;
   private final CANSparkMax _front_right_motor;
   private final CANSparkMax _back_right_motor;
   private final CANSparkMax _back_left_motor;
 
   /* Encoders */
   private final RelativeEncoder _front_left_encoder;
   private final RelativeEncoder _front_right_encoder;
   private final RelativeEncoder _back_right_encoder;
   private final RelativeEncoder _back_left_encoder;

   private final MecanumDrive _drive;


  public Drivetrain() {
    _front_left_motor = new CANSparkMax(DriveConstants.FRONTLEFT, MotorType.kBrushless);
    _front_right_motor = new CANSparkMax(DriveConstants.FRONTRIGHT, MotorType.kBrushless);
    _back_right_motor = new CANSparkMax(DriveConstants.BACKRIGHT, MotorType.kBrushless);
    _back_left_motor = new CANSparkMax(DriveConstants.BACKLEFT, MotorType.kBrushless);

    _front_left_motor.setInverted(true);
    _back_left_motor.setInverted(true);
    _back_right_motor.setInverted(false);
    _front_right_motor.setInverted(false);

    _front_left_encoder = _front_left_motor.getEncoder();
    _front_right_encoder = _front_right_motor.getEncoder();
    _back_right_encoder = _back_right_motor.getEncoder();
    _back_left_encoder = _back_left_motor.getEncoder();

    _drive = new MecanumDrive(_front_left_motor, _back_left_motor, _front_right_motor, _back_right_motor);

  }

  public void cDrive(double xSpeed, double ySpeed, double zRot) {
    _drive.driveCartesian(xSpeed, ySpeed, zRot);
  }

  @Override 
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
