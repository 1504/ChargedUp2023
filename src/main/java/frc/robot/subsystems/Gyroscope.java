package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

/**
 * Gyroscope subsystem
 * <p>
 * Warning: This class is a singleton. Use getInstance() to get the instance of
 * the Gyroscope subsystem
 * The constructor is private to prevent other classes from instantiating it.
 */
public class Gyroscope extends SubsystemBase {
    private static final AHRS _gyro = new AHRS(SerialPort.Port.kMXP);
    private static Gyroscope _instance = null;
    ShuffleboardTab telemetry = Shuffleboard.getTab("Telemetry");

    @Override
    public void periodic() {

    }

    /**
     * getInstance to provide a singleton instance of the Gyroscope subsystem
     * 
     * @return the instance of the Gyroscope subsystem
     */
    public static Gyroscope getInstance() {

        if (_instance == null) {
            _instance = new Gyroscope();
        }

        return _instance;
    }

    private Gyroscope() {
        // _gyro.reset(); // call gyro reset on first object instantiation
        _gyro.calibrate();
        SmartDashboard.putData("Gyro", _gyro);
    }

    public double getPitch() {
        return _gyro.getPitch();
    }

    public double getRoll() {
        return _gyro.getRoll();
    }

    public double getYaw() {
        return _gyro.getYaw();
    }

    public double getDisplacementX() {
        return _gyro.getDisplacementX();
    }

    public double getDisplacementY() {
        return _gyro.getDisplacementY();
    }

    public double getDisplacementZ() {
        return _gyro.getDisplacementZ();
    }

    public Rotation2d getRotation2d() {
        return _gyro.getRotation2d();
    }

    public Rotation2d getYawRotation() {
        return (Constants.DriveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - getYaw())
                : Rotation2d.fromDegrees(getYaw());
    }

    public void reset() {
        _gyro.reset();
        System.out.println("Gyro Reset");
    }

}