package frc.robot.controlboard.profiles;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.IOConstants;
import frc.robot.controlboard.IDriveProfile;

public class JoystickProfile implements IDriveProfile{

    private final Joystick j1;
    private final Joystick j2;
    private static JoystickProfile _instance = null;
    ShuffleboardTab controlBoard;

    GenericEntry throttleEntry;
    GenericEntry rightEntry;
    GenericEntry rotEntry;
    GenericEntry throttlEntryFiltered;
    GenericEntry rightEntryFiltered;
    GenericEntry rotEntryFiltered;

    LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);

    /**
     * Singleton pattern to ensure only one instance of the Joystick is created
     * 
     * @return JoystickProfile instance
     */
    public static JoystickProfile getInstance() {

        if (_instance == null) {
            _instance = new JoystickProfile();
            
        }

        return _instance;
    }

    private JoystickProfile() {
        j1 = new Joystick(IOConstants.JOYSTICK_ONE);
        j2 = new Joystick(IOConstants.JOYSTICK_TWO);
    }

    /**
     * Gets the throttle value from the joystick
     * 
     * @return throttle value
     */

    @Override
    public double getThrottle() {
        return j1.getRawAxis(1);
    }

    /**
     * Gets the left and right values from the joystick
     * 
     * @return left and right value
     */
    @Override
    public double getRight() {
        return j1.getRawAxis(0);
    }

    /**
     * 
     * Gets the rotation value from the joystick
     * 
     * @return rotation value
     * 
     */
    @Override
    public double getRot() {
        return j2.getRawAxis(0);
    }

    @Override
    public boolean getRawButtonPressed(int button) {
        return j1.getRawButtonPressed(button);
    }

    @Override
    public void shuffleboardInit() {
    }

    @Override
    public void shuffleboardUpdate() {
    }

    @Override
    public GenericHID getLeftController() { // Use Joystick #1 for arm control
        return j1;
    }

    @Override
    public GenericHID getRightController() { // Use Joystick #2 for gripper control
        return j2;
    }

}
