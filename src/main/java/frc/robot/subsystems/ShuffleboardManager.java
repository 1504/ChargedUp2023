// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    ShuffleboardTab PIDdrive;

    ShuffleboardTab PIDarm;

    private GenericEntry frontLeftEncoder;
    private GenericEntry frontRightEncoder;
    private GenericEntry backRightEncoder;
    private GenericEntry backLeftEncoder;

    private GenericEntry gyroPitch;
    private GenericEntry gyroYaw;
    private GenericEntry gyroRoll;
    private GenericEntry resetGyro;

    private GenericEntry armPosition;
    private GenericEntry resetArmPosition;

    private Drivetrain _drive = Drivetrain.getInstance();
    private Gyroscope _gyro = Gyroscope.getInstance();
    private Arm _arm = Arm.getInstance();

    private ShuffleboardManager() {
        shuffleboardInit();
    }

    public void shuffleboardInit() {

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

        // PID drive stuff

        // Optional: add functionality to modify PID values on the fly
        PIDdrive = Shuffleboard.getTab("PID Drive Tuning");
        PIDdrive.add("PID", _drive.getWheelPid())
                .withPosition(2, 0);
        PIDdrive.add("front left pid", _drive.getFrontLeftPid())
                .withPosition(0, 0);
        PIDdrive.add("front right pid", _drive.getFrontRightPid())
                .withPosition(1, 0);
        PIDdrive.add("back left pid", _drive.getBackLeftPid())
                .withPosition(0, 2);
        PIDdrive.add("back right pid", _drive.getBackRightPid())
                .withPosition(1, 2);
        PIDdrive.add("x pid", _drive.getXPid())
                .withPosition(3, 0);
        PIDdrive.add("y pid", _drive.getYPid())
                .withPosition(4, 0);

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
    }

    public void shuffleboardUpdate() {

        // updates encoder values
        frontLeftEncoder.setDouble(_drive.getFrontLeftMeters());
        frontRightEncoder.setDouble(_drive.getFrontRightMeters());
        backRightEncoder.setDouble(_drive.getBackRightMeters());
        backLeftEncoder.setDouble(_drive.getBackLeftMeters());

        // updates gyro values
        gyroPitch.setDouble(_gyro.getPitch());
        gyroYaw.setDouble(_gyro.getYaw());
        gyroRoll.setDouble(_gyro.getRoll());
        // if gyro reset button is pressed, reset gyro
        if (resetGyro.getBoolean(true)) {
            _gyro.reset();
        }
        // updates arm position
        armPosition.setDouble(_arm.getArmDistance());
        // if arm reset button is pressed, reset arm position
        if (resetArmPosition.getBoolean(true)) {
                armPosition.setDouble(0);
        }
    }

    @Override
    public void periodic() {
        shuffleboardUpdate();
    }
}
