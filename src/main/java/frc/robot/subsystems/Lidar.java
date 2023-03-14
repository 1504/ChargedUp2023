// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// dio pins 0,1,2 are used for the lidar, we read their values to determine the zone
import edu.wpi.first.wpilibj.DigitalInput;

public class Lidar extends SubsystemBase {
    private DigitalInput bit0;
    private DigitalInput bit1;
    private DigitalInput bit2;

    private Action currentAction;

    // current action enum
    public enum Action {
        NONE, FORWARD, BACKWARD, LEFT, RIGHT, GRIP, FORWARD_LEFT, FORWARD_RIGHT
    }

    public Lidar() {
        // initialize the DIO pins
        bit0 = new DigitalInput(0);
        bit1 = new DigitalInput(1);
        bit2 = new DigitalInput(2);
        // initialize the current action to none
        currentAction = Action.NONE;
    }

    public Action getAction() {
        return currentAction;
    }

    @Override
    public void periodic() {
        // call the calculateZone method to determine the zone, and assign the action
        // based on the zone
        switch (calculateZone()) {
            case 0:
                currentAction = Action.NONE;
                break;
            case 1:
                currentAction = Action.FORWARD_LEFT;
                break;
            case 2:
                currentAction = Action.FORWARD;
                break;
            case 3:
                currentAction = Action.FORWARD_RIGHT;
                break;
            case 4:
                currentAction = Action.LEFT;
                break;
            case 5:
                currentAction = Action.GRIP;
                break;
            case 6:
                currentAction = Action.RIGHT;
                break;
            case 7:
                currentAction = Action.BACKWARD;
                break;
            default:
                currentAction = Action.NONE;
                break;
        }
    }

    /**
     * @brief Calculates the current zone based on the bits read from the lidar DIO
     *        pins
     * @return The current zone, in the range 0-7
     */

    private int calculateZone() {
        boolean[] bits = getBits();
        int zone = 0;
        if (bits[0]) {
            zone += 1;
        }
        if (bits[1]) {
            zone += 2;
        }
        if (bits[2]) {
            zone += 4;
        }
        return zone;
    }

    /**
     * @brief Gets the individual bits from the DIO pins
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
