package frc.robot.controlboard.profiles;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.IOConstants;
import frc.robot.controlboard.IDriveProfile;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.GenericEntry;

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

    @Override
    public double getThrottle() {
        
        return j1.getRawAxis(1);
    }

    @Override
    public double getRight() {
        
        return j1.getRawAxis(0) * (-1);
    }

    @Override
    public double getRot() {
        return j2.getX();
    }

    @Override
    public boolean getRawButtonPressed(int button) {
        return j1.getRawButtonPressed(button);
    }

    @Override
    public void shuffleboardInit() {
        /*
        controlBoard = Shuffleboard.getTab("Control Board");
        throttleEntry = controlBoard.add("Throttle", 0)
            .withPosition(0, 0)
            .withSize(3,3)
            .getEntry();
        //rightEntry = controlBoard.add("Right", 0).getEntry();
        //rotEntry = controlBoard.add("Rot", 0).getEntry();
        throttlEntryFiltered = controlBoard.add("Throttle Filtered", 0)
            .withPosition(3, 0)
            .withSize(3,3)
            .getEntry();
        //rightEntryFiltered = controlBoard.add("Right Filtered", 0).getEntry();
        //rotEntryFiltered = controlBoard.add("Rot Filtered", 0).getEntry();
        */
    }

    @Override
    public void shuffleboardUpdate() {
        /*
        throttleEntry.setDouble(j1.getRawAxis(1) * (-1));
        //rightEntry.setDouble(getRight());
        //rotEntry.setDouble(getRot());
        throttlEntryFiltered.setDouble(filter.calculate(j1.getRawAxis(1) * (-1)));
        //rightEntryFiltered.setDouble(filter.calculate(getRight()));
        //rotEntryFiltered.setDouble(filter.calculate(getRot()));
        */
    }

    @Override
    public GenericHID getArmController() { // Use Joystick #1 for arm control
        return j1;
    }

    @Override
    public GenericHID getGripperController() { // Use Joystick #2 for gripper control
        return j2;
    }


}
