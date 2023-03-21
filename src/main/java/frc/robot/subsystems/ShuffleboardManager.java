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
import frc.robot.commands.arm.ResetArmPosition;
import frc.robot.commands.balance.AutoBalance;
import frc.robot.commands.drive.ResetGyro;
import frc.robot.commands.gripper.ToggleAuto;

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

        // shuffle board
        ShuffleboardTab telemetry;

        PowerDistribution m_pdp = new PowerDistribution();

        ShuffleboardTab PIDdrive;

        ShuffleboardTab PIDarm;

        private GenericEntry frontLeftEncoder;
        private GenericEntry frontRightEncoder;
        private GenericEntry backRightEncoder;
        private GenericEntry backLeftEncoder;

        private GenericEntry xPos;
        private GenericEntry yPos;
        private GenericEntry zPos;

        private GenericEntry gyroPitch;
        private GenericEntry gyroYaw;
        private GenericEntry gyroRoll;
        private GenericEntry resetGyro;

        private GenericEntry armPosition;
        private GenericEntry resetArmPosition;

        private GenericEntry RobotPosition;

        private Drivetrain _drive = Drivetrain.getInstance();
        private Gyroscope _gyro = Gyroscope.getInstance();
        private Arm _arm = Arm.getInstance();

        private ShuffleboardManager() {
                shuffleboardInit();
                SmartDashboard.putData("Auto Balance", new AutoBalance());
                SmartDashboard.putData("Reset Arm", new ResetArmPosition());
                SmartDashboard.putData("Reset Gyro", new ResetGyro());
                SmartDashboard.putData("Toggle Auto", new ToggleAuto());
        }

        public void shuffleboardInit() {
                try {

                        // encoders and gyro stuff
                        telemetry = Shuffleboard.getTab("Telemetry");

                        frontLeftEncoder = telemetry.add("Front Left Encoder", 0).withPosition(0, 0).withSize(2, 2).withWidget(BuiltInWidgets.kTextView).getEntry();
                        frontRightEncoder = telemetry.add("Front Right Encoder", 0).withPosition(2, 0).withSize(2, 2).withWidget(BuiltInWidgets.kTextView).getEntry();
                        backRightEncoder = telemetry.add("Back Right Encoder", 0).withPosition(0, 2).withSize(2, 2).withWidget(BuiltInWidgets.kTextView).getEntry();
                        backLeftEncoder = telemetry.add("Back Left Encoder", 0).withPosition(2, 2).withSize(2, 2).withWidget(BuiltInWidgets.kTextView).getEntry();
                        gyroPitch = telemetry.add("Gyro Pitch", 0).withWidget(BuiltInWidgets.kNumberBar).getEntry();
                        gyroYaw = telemetry.add("Gyro Yaw", 0).withWidget(BuiltInWidgets.kNumberBar).getEntry();
                        gyroRoll = telemetry.add("Gyro Roll", 0).withWidget(BuiltInWidgets.kNumberBar).getEntry();
                        RobotPosition = telemetry.add("Robot Position", "Null").withWidget(BuiltInWidgets.kTextView).getEntry();
                        // PID drive stuff

                        // Optional: add functionality to modify PID values on the fly
                        PIDdrive = Shuffleboard.getTab("PID Drive Tuning");
                        PIDdrive.add("front left pid", _drive.getFrontLeftPid()).withPosition(0, 0);
                        PIDdrive.add("front right pid", _drive.getFrontRightPid()).withPosition(1, 0);
                        PIDdrive.add("back left pid", _drive.getBackLeftPid()).withPosition(0, 2);
                        PIDdrive.add("back right pid", _drive.getBackRightPid()).withPosition(1, 2);

                        // Arm stuff
                        PIDarm = Shuffleboard.getTab("Arm PID tuning");
                        armPosition = PIDarm.add("Arm Position", 0).withPosition(1, 0).withSize(3, 3).getEntry();
                        PIDarm.add("arm pid", _arm.getArmPid()).withPosition(0, 0);
                        resetArmPosition = PIDarm.add("Reset Arm", false).withWidget(BuiltInWidgets.kToggleButton).withPosition(5, 0).withSize(1, 1).getEntry();
                } catch (Exception e) {
                        System.out.println("ShuffleboardManager: " + e);
                }
/*
                // encoders and gyro stuff
                telemetry = Shuffleboard.getTab("Telemetry");

                frontLeftEncoder = telemetry.add("Front Left Encoder", 0)
                                .withPosition(0, 0)
                                .withSize(2, 2)
                                .withWidget(BuiltInWidgets.kTextView)
                                .getEntry();
                frontRightEncoder = telemetry.add("Front Right Encoder", 0)
                                .withPosition(2, 0)
                                .withSize(2, 2)
                                .withWidget(BuiltInWidgets.kTextView)
                                .getEntry();
                backRightEncoder = telemetry.add("Back Right Encoder", 0)
                                .withPosition(0, 2)
                                .withSize(2, 2)
                                .withWidget(BuiltInWidgets.kTextView)
                                .getEntry();
                backLeftEncoder = telemetry.add("Back Left Encoder", 0)
                                .withPosition(2, 2)
                                .withSize(2, 2)
                                .withWidget(BuiltInWidgets.kTextView)
                                .getEntry();
                gyroPitch = telemetry.add("Gyro Pitch", 0)
                                .withWidget(BuiltInWidgets.kNumberBar)
                                .getEntry();
                gyroYaw = telemetry.add("Gyro Yaw", 0)
                                .withWidget(BuiltInWidgets.kNumberBar)
                                .getEntry();
                gyroRoll = telemetry.add("Gyro Roll", 0)
                                .withWidget(BuiltInWidgets.kNumberBar)
                                .getEntry();
                resetGyro = telemetry.add("Reset Gyro", false)
                                .withWidget(BuiltInWidgets.kToggleButton)
                                .withPosition(8, 0)
                                .withSize(1, 1)
                                .getEntry();
                                */

                //Robot position stuff
                xPos = telemetry.add("X Position", 0)
                        .withPosition(4, 1)
                        .withSize(3, 1)
                        .getEntry();
                yPos = telemetry.add("Y Position", 0)
                        .withPosition(4, 2)
                        .withSize(3, 1)
                        .getEntry();
                zPos = telemetry.add("Z Position", 0)
                        .withPosition(4, 3)
                        .withSize(3, 1)
                        .getEntry();
                // PID drive stuff
                /*

                // Optional: add functionality to modify PID values on the fly
                PIDdrive = Shuffleboard.getTab("PID Drive Tuning");
                PIDdrive.add("front left pid", _drive.getFrontLeftPid())
                                .withPosition(0, 0);
                PIDdrive.add("front right pid", _drive.getFrontRightPid())
                                .withPosition(1, 0);
                PIDdrive.add("back left pid", _drive.getBackLeftPid())
                                .withPosition(0, 2);
                PIDdrive.add("back right pid", _drive.getBackRightPid())
                                .withPosition(1, 2);
              

                // Arm stuff
                PIDarm = Shuffleboard.getTab("Arm PID tuning");

                armPosition = PIDarm.add("Arm Position", 0)
                                .withPosition(1, 0)
                                .withSize(3, 3)
                                .getEntry();
                PIDarm.add("arm pid", _arm.getArmPid())
                                .withPosition(0, 0);

                resetArmPosition = PIDarm.add("Reset Arm", false)
                                .withWidget(BuiltInWidgets.kToggleButton)
                                .withPosition(5, 0)
                                .withSize(1, 1)
                                .getEntry();
                                                  */

        }

        public void shuffleboardUpdate() {

                // updates encoder values
                /*
                frontLeftEncoder.setDouble(_drive.getFrontLeftVelocity());
                frontRightEncoder.setDouble(_drive.getFrontRightVelocity());
                backRightEncoder.setDouble(_drive.getBackRightVelocity());
                backLeftEncoder.setDouble(_drive.getBackLeftVelocity());

                 */
                frontLeftEncoder.setDouble(_drive.getFrontLeftDistance());
                frontRightEncoder.setDouble(_drive.getFrontRightDistance());
                backRightEncoder.setDouble(_drive.getBackRightDistance());
                backLeftEncoder.setDouble(_drive.getBackLeftDistance());
                RobotPosition.setString(_drive.getPoseEstimate().toString());

                // Get the voltage going into the PDP, in Volts.
                // The PDP returns the voltage in increments of 0.05 Volts.
                double voltage = m_pdp.getVoltage();
                SmartDashboard.putNumber("Voltage", voltage);

                // Retrieves the temperature of the PDP, in degrees Celsius.
                double temperatureCelsius = m_pdp.getTemperature();
                SmartDashboard.putNumber("Temperature", temperatureCelsius);

                // Get the total current of all channels.
                double totalCurrent = m_pdp.getTotalCurrent();
                SmartDashboard.putNumber("Total Current", totalCurrent);

                // Get the total power of all channels.
                // Power is the bus voltage multiplied by the current with the units Watts.
                double totalPower = m_pdp.getTotalPower();
                SmartDashboard.putNumber("Total Power", totalPower);

                // Get the total energy of all channels.
                // Energy is the power summed over time with units Joules.
                double totalEnergy = m_pdp.getTotalEnergy();
                SmartDashboard.putNumber("Total Energy", totalEnergy);
                // updates robot positions
                xPos.setDouble(_drive.getPose().getX());
                yPos.setDouble(_drive.getPose().getY());
                //zPos.setDouble(_drive.getPose())

                // updates gyro values
                gyroPitch.setDouble(_gyro.getPitch());
                gyroYaw.setDouble(_gyro.getYaw());
                gyroRoll.setDouble(_gyro.getRoll());
                // updates arm position
                armPosition.setDouble(_arm.getArmDistance());

                /*
                 * Warning: This will reset the arm encoder position to 0. This may cause the
                 * arm to move to the bottom of its range. This is not recommended unless you
                 * know what you're doing
                 */
                if (resetArmPosition.getBoolean(true)) {
                        _arm.resetArmEncoderPosition(); // Please don't use this unless you know what you're doing
                }
                SmartDashboard.putBoolean("Auto Status", _arm.getAutoStatus());
        }

        @Override
        public void periodic() {
                shuffleboardUpdate();
        }
}
