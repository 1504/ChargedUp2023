package frc.robot.controlboard;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.controlboard.profiles.DDRProfile;
import frc.robot.controlboard.profiles.JoystickProfile;
import frc.robot.controlboard.profiles.XBoxControllerProfile;

public class ControlBoard {

    private static ControlBoard _instance = null;
    private IDriveProfile dboard;
    private final SendableChooser<IDriveProfile> mProfileChooser;

    ShuffleboardTab p_tab = Shuffleboard.getTab("Pregame");

    /**
     * Singleton pattern to ensure only one instance of the ControlBoard is created
     * 
     * @return ControlBoard instance
     */

    public static ControlBoard getInstance() {

        if (_instance == null) {
            _instance = new ControlBoard();
        }

        return _instance;
    }

    private ControlBoard() {
        mProfileChooser = new SendableChooser<>();

        mProfileChooser.setDefaultOption("Joystick", JoystickProfile.getInstance());
        mProfileChooser.addOption("XBox", XBoxControllerProfile.getInstance());
        mProfileChooser.addOption("DDR", DDRProfile.getInstance());

        p_tab.add("Profile", mProfileChooser)
                .withPosition(0, 0)
                .withSize(3, 1);
        dboard = getProfile();

    }

    /**
     * Updates the drive profile
     */
    public void updateProfile() {
        dboard = getProfile();
        System.out.println(getProfile());
    }

    /**
     * Updates the drive profile based on given profile on shuffleboard
     * 
     * @return the selected profile
     */
    public IDriveProfile getProfile() {
        return mProfileChooser.getSelected();
    }

    public double getThrottle() {
        return dboard.getThrottle();
    }

    public double getRight() {
        return dboard.getRight();
    }

    public double getRot() {
        return dboard.getRot();
    }

    public boolean getRawButtonPressed(int button) {
        return dboard.getRawButtonPressed(button);
    }

    public void shuffleboardInit() {
        dboard.shuffleboardInit();
    }

    public void shuffleboardUpdate() {
        dboard.shuffleboardUpdate();
    }


    public GenericHID getLeftController() {
        return dboard.getLeftController();
    }

    public GenericHID getRightController() {
        return dboard.getRightController();
    }
}
