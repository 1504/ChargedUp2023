package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Gyroscope extends SubsystemBase {
    private static final AHRS _gyro = new AHRS(SerialPort.Port.kMXP);
    ShuffleboardTab telemetry = Shuffleboard.getTab("Telemetry");
    GenericEntry gyroPitch = telemetry.add("Gyro Pitch", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();
    GenericEntry gyroYaw = telemetry.add("Gyro Yaw", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();
    GenericEntry gyroRoll = telemetry.add("Gyro Roll", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();

    // create button on shuffleboard to reset gyro
    GenericEntry resetGyro = telemetry.add("Reset Gyro", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(8, 0)
            .withSize(1, 1)
            .getEntry();

    @Override
    public void periodic() {
        gyroPitch.setDouble(getPitch());
        gyroYaw.setDouble(getYaw());
        gyroRoll.setDouble(getRoll());
        // if gyro reset button is pressed, reset gyro
        if (resetGyro.getBoolean(true)) {
            _gyro.reset();
        }

    }

    public Gyroscope() {
        // _gyro.reset();
    }

    public static double getPitch() {
        return _gyro.getPitch();
    }

    public static double getRoll() {
        return _gyro.getRoll();
    }

    public static double getYaw() {
        return _gyro.getYaw();
    }

    public static Rotation2d getRotation2d() {
        return _gyro.getRotation2d();
    }

    public static void reset() {
        _gyro.reset();
        System.out.println("Gyro Reset");
    }

}