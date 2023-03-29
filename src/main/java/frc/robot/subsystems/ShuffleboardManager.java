// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.resets.ResetArmPosition;
import frc.robot.commands.balance.AutoBalance;
import frc.robot.commands.resets.ResetEncoders;
import frc.robot.commands.resets.ResetGyro;
import frc.robot.commands.resets.ResetOdometry;
import frc.robot.commands.arm.ToggleAuto;

/**
 * ShuffleboardManager subsystem
 * <p>
 * Warning: This class is a singleton. Use getInstance() to get the instance of
 * the ShuffleboardManager subsystem
 * The constructor is private to prevent other classes from instantiating it.
 */
public class ShuffleboardManager extends SubsystemBase {

        private static ShuffleboardManager _instance = null;

        /**
         * getInstance to provide a singleton instance of the ShuffleboardManager
         * subsystem
         *
         * @return the instance of the ShuffleboardManager subsystem
         */
        public static ShuffleboardManager getInstance() {
                if (_instance == null) {
                        _instance = new ShuffleboardManager();
                }
                return _instance;
        }

        private final Drivetrain _drive = Drivetrain.getInstance();
        private final Gyroscope _gyro = Gyroscope.getInstance();
        private final Arm _arm = Arm.getInstance();

        ShuffleboardTab telemetry;

        PowerDistribution m_pdp = new PowerDistribution();

        ShuffleboardTab PID_Drive;

        ShuffleboardTab PID_Arm;

        private GenericEntry frontLeftEncoder;
        private GenericEntry frontRightEncoder;
        private GenericEntry backRightEncoder;
        private GenericEntry backLeftEncoder;
        private GenericEntry gyroPitch;
        private GenericEntry gyroYaw;
        private GenericEntry gyroRoll;
        private GenericEntry armPosition;
        private GenericEntry RobotPosition;
        private GenericEntry xPos;
        private GenericEntry yPos;

        private ShuffleboardManager() {
                shuffleboardInit();
                SmartDashboard.putData("Auto Balance", new AutoBalance());
                SmartDashboard.putData("Reset Arm", new ResetArmPosition());
                SmartDashboard.putData("Reset Gyro", new ResetGyro());
                SmartDashboard.putData("Toggle Auto", new ToggleAuto());
                SmartDashboard.putData("Reset Encoders", new ResetEncoders());
                SmartDashboard.putData("Reset Odometry", new ResetOdometry());
                SmartDashboard.putData("Reset Arm", new ResetArmPosition());
                SmartDashboard.putData("Reset All", Commands.run(() -> {
                        new ResetArmPosition().schedule();
                        new ResetGyro().schedule();
                        new ResetEncoders().schedule();
                        new ResetOdometry().schedule();
                }));
                SmartDashboard.putBoolean("Coast", _drive.getCoasting());
                SmartDashboard.putData("Angle PID", _drive.getAnglePID());
        }



        public void shuffleboardInit() {
                try {
                        // encoders and gyro stuff
                        telemetry = Shuffleboard.getTab("Telemetry");
                        frontLeftEncoder = telemetry.add("Front Left Encoder", 0).withPosition(0, 0).withSize(2, 2).withWidget(BuiltInWidgets.kTextView).getEntry();
                        frontRightEncoder = telemetry.add("Front Right Encoder", 0).withPosition(2, 0).withSize(2, 2).withWidget(BuiltInWidgets.kTextView).getEntry();
                        backRightEncoder = telemetry.add("Back Right Encoder", 0).withPosition(0, 2).withSize(2, 2).withWidget(BuiltInWidgets.kTextView).getEntry();
                        backLeftEncoder = telemetry.add("Back Left Encoder", 0).withPosition(2, 2).withSize(2, 2).withWidget(BuiltInWidgets.kTextView).getEntry();
                        gyroPitch = telemetry.add("Gyro Pitch", _gyro.getPitch()).withWidget(BuiltInWidgets.kNumberBar).getEntry();
                        gyroYaw = telemetry.add("Gyro Yaw", _gyro.getYaw()).withWidget(BuiltInWidgets.kNumberBar).getEntry();
                        gyroRoll = telemetry.add("Gyro Roll", _gyro.getRoll()).withWidget(BuiltInWidgets.kNumberBar).getEntry();
                        telemetry.add("Reset Gyro", new ResetGyro()).withPosition(7, 0).withSize(1, 1);
                        RobotPosition = telemetry.add("Robot Position", "Null").withWidget(BuiltInWidgets.kTextView).getEntry();
                        xPos = telemetry.add("X Position", 0)
                                .withPosition(4, 1)
                                .withSize(3, 1)
                                .withWidget(BuiltInWidgets.kTextView)
                                .getEntry();
                        yPos = telemetry.add("Y Position", 0)
                                .withPosition(4, 2)
                                .withSize(3, 1)
                                .withWidget(BuiltInWidgets.kTextView)
                                .getEntry();
                        // PID drive stuff

                        PID_Drive = Shuffleboard.getTab("PID Drive Tuning");
                        PID_Drive.add("front left pid", _drive.getFrontLeftPid()).withPosition(0, 0);
                        PID_Drive.add("front right pid", _drive.getFrontRightPid()).withPosition(1, 0);
                        PID_Drive.add("back left pid", _drive.getBackLeftPid()).withPosition(0, 2);
                        PID_Drive.add("back right pid", _drive.getBackRightPid()).withPosition(1, 2);
                        PID_Drive.add("Theta PID", _drive.getAnglePID());

                        // Arm stuff
                        PID_Arm = Shuffleboard.getTab("Arm PID tuning");
                        armPosition = PID_Arm.add("Arm Position", 0).withPosition(1, 0).withSize(3, 3).getEntry();
                        PID_Arm.add("arm pid", _arm.getArmPid()).withPosition(0, 0);
                        // resetArmPosition = PIDarm.add("Reset Arm", false).withWidget(BuiltInWidgets.kToggleButton).withPosition(5, 0).withSize(1, 1).getEntry();
                        PID_Arm.add("Reset Arm Position", new ResetArmPosition()).withPosition(4, 0).withSize(1, 1);
                } catch (Exception e) {
                        System.out.println("ShuffleboardManager: " + e);
                }

        }

        public void shuffleboardUpdate() {
                frontLeftEncoder.setDouble(_drive.getFrontLeftVelocity());
                frontRightEncoder.setDouble(_drive.getFrontRightVelocity());
                backRightEncoder.setDouble(_drive.getBackRightVelocity());
                backLeftEncoder.setDouble(_drive.getBackLeftVelocity());
                RobotPosition.setString(_drive.getPoseEstimate().toString());
                gyroPitch.setDouble(_gyro.getPitch());
                gyroYaw.setDouble(_gyro.getYaw());
                gyroRoll.setDouble(_gyro.getRoll());
                armPosition.setDouble(_arm.getArmDistance()); // get arm position
                xPos.setDouble(_drive.getPose().getX());
                yPos.setDouble(_drive.getPose().getY());
                SmartDashboard.putNumber("Voltage", m_pdp.getVoltage());
                SmartDashboard.putNumber("Total Current", m_pdp.getTotalCurrent());
                SmartDashboard.putBoolean("Auto Status", _arm.getAutoStatus()); // get auto status
                SmartDashboard.putBoolean("Coast", _drive.getCoasting());
        }

        @Override
        public void periodic() {
                shuffleboardUpdate();
        }
}
