package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.subsystems.Arm;

public class PenguinDrive extends MecanumDrive {

    private volatile double[] _orbit_point = { 1.0 , 0.0}; // -1.15}; //{0.0, 1.15};
    private double[] _orbit_magic_numbers = new double[6];
    private static final Arm _arm = Arm.getInstance();
    private volatile double _last_orbit_extend = 0;

    public PenguinDrive(MotorController frontLeftMotor, MotorController rearLeftMotor, MotorController frontRightMotor,
            MotorController rearRightMotor) {
        super(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
		set_orbit_point(_orbit_point);
    }

    public void driveCartesian(double xSpeed, double ySpeed, double zRotation) {
        driveCartesian(xSpeed, ySpeed, zRotation, new Rotation2d());
    }

    public void driveCartesian(double xSpeed, double ySpeed, double zRotation, boolean orbit) {
        if(Math.abs(_arm.getArmDistance() - _last_orbit_extend) > 5.0)
        {
            double[] point = {1.0 + 4.2 * _arm.getArmDistance()/100.0,0};
            set_orbit_point(point);
        }
        double[] input = {ySpeed, xSpeed, zRotation};
        double[] corrected = orbit_point(input);
        driveCartesian(corrected[1], corrected[0], corrected[2], new Rotation2d());
    }

    	/**
	 * Orbit point changes the pivot point that the robot rotates around when
	 * turning. borrowed from @cowplex
	 */
	private double[] orbit_point(double[] input) {
		/*
		 * double x = _orbit_point[0]; double y = _orbit_point[1];
		 * 
		 * double[] k = { y - 1, y + 1, 1 - x, -1 - x };
		 * 
		 * double p = Math.sqrt((k[0] * k[0] + k[2] * k[2]) / 2) * Math.cos((Math.PI /
		 * 4) + Math.atan2(k[0], k[2])); double r = Math.sqrt((k[1] * k[1] + k[2] *
		 * k[2]) / 2) * Math.cos(-(Math.PI / 4) + Math.atan2(k[1], k[2])); double q =
		 * -Math.sqrt((k[1] * k[1] + k[3] * k[3]) / 2) * Math.cos((Math.PI / 4) +
		 * Math.atan2(k[1], k[3]));
		 */

		double p = _orbit_magic_numbers[3];
		double r = _orbit_magic_numbers[4];
		double q = _orbit_magic_numbers[5];

		double[] corrected = new double[3];
		corrected[0] = (input[2] * r + (input[0] - input[2]) * q + input[0] * p) / (q + p);
		corrected[1] = (-input[2] * r + input[1] * q - (-input[1] - input[2]) * p) / (q + p);
		corrected[2] = (2 * input[2]) / (q + p);
		return corrected;
	}

	public void set_orbit_point(double[] orbit_point) {
		_orbit_point = orbit_point;

		double x = _orbit_point[0];
		double y = _orbit_point[1];

		double[] k = { y - 1, y + 1, 1 - x, -1 - x };

		double p = Math.sqrt((k[0] * k[0] + k[2] * k[2]) / 2) * Math.cos((Math.PI / 4) + Math.atan2(k[0], k[2]));
		double r = Math.sqrt((k[1] * k[1] + k[2] * k[2]) / 2) * Math.cos(-(Math.PI / 4) + Math.atan2(k[1], k[2]));
		double q = -Math.sqrt((k[1] * k[1] + k[3] * k[3]) / 2) * Math.cos((Math.PI / 4) + Math.atan2(k[1], k[3]));

		_orbit_magic_numbers[0] = k[0];
		_orbit_magic_numbers[1] = k[1];
		_orbit_magic_numbers[2] = k[2];
		_orbit_magic_numbers[3] = p;
		_orbit_magic_numbers[4] = r;
		_orbit_magic_numbers[5] = q;
	}
}
