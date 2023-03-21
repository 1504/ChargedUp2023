// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DIOPins;
// dio pins 0,1,2 are used for the lidar, we read their values as bits to determine the zone
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Lidar subsystem
 * <p>
 * Warning: This class is a singleton. Use getInstance() to get the instance of
 * the Lidar subsystem
 * The constructor is private to prevent other classes from instantiating it.
 */
public class Lidar extends SubsystemBase {
    private final DigitalInput bit0;
    private final DigitalInput bit1;
    private final DigitalInput bit2;

    private Action currentAction;

    private static Lidar _instance = null;

    /**
     * getInstance to provide a singleton instance of the Lidar subsystem
     * 
     * @return the instance of the Lidar subsystem
     */
    public static Lidar getInstance() {
        if (_instance == null) {
            _instance = new Lidar();
        }
        return _instance;
    }

    /**
     * Current action to be taken by the robot
     */
    public enum Action {
        NONE, FORWARD, BACKWARD, LEFT, RIGHT, GRIP, FORWARD_LEFT, FORWARD_RIGHT
    }

    private Lidar() {
        // initialize the DIO pins
        bit0 = new DigitalInput(DIOPins.LIDAR_FIRST_BIT);
        bit1 = new DigitalInput(DIOPins.LIDAR_SECOND_BIT);
        bit2 = new DigitalInput(DIOPins.LIDAR_THIRD_BIT);
        // initialize the current action to none
        currentAction = Action.NONE;
    }

    /**
     * Called periodically to get the current action
     * @return The current action
     */
    public Action getAction() {
        return currentAction;
    }

    @Override
    public void periodic() {
        // call the calculateZone method to determine the zone, and assign the action
        // based on the zone
        switch (calculateZone()) {
            case 0:
                currentAction = Action.NONE; // not detecting anything
                break;
            case 1:
                currentAction = Action.FORWARD_LEFT; // robot action should be to move left first, until object in zone
                                                     // 2, then move forward
                break;
            case 2:
                currentAction = Action.FORWARD; // move forward until object in zone 5, then grip
                break;
            case 3:
                currentAction = Action.FORWARD_RIGHT; // robot action should be to move right first, until object in
                                                      // zone 2, then move forward
                break;
            case 4:
                currentAction = Action.LEFT; // move left until object in zone 5, then grip
                break;
            case 5:
                currentAction = Action.GRIP; // grip the object
                break;
            case 6:
                currentAction = Action.RIGHT; // move right until object in zone 5, then grip
                break;
            case 7:
                currentAction = Action.BACKWARD; // move backward until object in zone 5, then grip
                break;
            default:
                currentAction = Action.NONE; // although this should never happen, if it does, set the action to none
                break;
        }
    }

    /**
     * Calculates the current zone based on the bits read from the lidar DIO
     *        pins
     * @return The current zone, in the range 0-7
     */

    private int calculateZone() {
        boolean[] bits = getBits();
        int zone = 0;
        if (bits[0]) {
            zone += 1; // 1<<0
        }
        if (bits[1]) {
            zone += 2; // 1<<1
        }
        if (bits[2]) {
            zone += 4; // 1<<2
        }
        return zone;
    }

    /**
     * Gets the individual bits from the DIO pins
     * @return An array of boolean values representing the bits
     */
    private boolean[] getBits() {
        boolean[] bits = new boolean[3];
        bits[0] = bit0.get();
        bits[1] = bit1.get();
        bits[2] = bit2.get();
        return bits;
    }

}
