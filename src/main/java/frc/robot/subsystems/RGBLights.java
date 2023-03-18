// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PWMPins;
import edu.wpi.first.wpilibj.PWM;

/**
 * RGBLights subsystem
 * <p>
 * Warning: This class is a singleton. Use getInstance() to get the instance of
 * the RGBLights subsystem
 * The constructor is private to prevent other classes from instantiating it.
 */
public class RGBLights extends SubsystemBase {

    private static RGBLights _instance = null;

    /**
     * getInstance to provide a singleton instance of the RGBLights subsystem
     * 
     * @return the instance of the RGBLights subsystem
     */
    public static RGBLights getInstance() {
        if (_instance == null) {
            _instance = new RGBLights();
        }
        return _instance;
    }

    private PWM red_pwd;
    private PWM green_pwd;
    private PWM blue_pwd;

    private RGBLights() {
        red_pwd = new PWM(PWMPins.RED_LED);
        green_pwd = new PWM(PWMPins.GREEN_LED);
        blue_pwd = new PWM(PWMPins.BLUE_LED);
    }

    // private int num = 0;

    @Override
    public void periodic() {
        // updateLights(num);
        // num++;
    }

    // to activate a light, set the pin to false (0v)

    public void updateLights(int num) {
        red_pwd.setRaw((int) ((num % 2550) / 10));
        green_pwd.setRaw((int) ((num % 2550) / 10));
        blue_pwd.setRaw((int) ((num % 2550) / 10));
        // System.out.println((int) ((num % 2550) / 10));
    }

    public boolean setFromHex(String hex) {
        // remove # if it exists
        if (hex.charAt(0) == '#') {
            hex = hex.substring(1);
        }
        if (hex.length() != 6) {
            return false;
        } else {
            int r = Integer.parseInt(hex.substring(0, 2), 16);
            int g = Integer.parseInt(hex.substring(2, 4), 16);
            int b = Integer.parseInt(hex.substring(4, 6), 16);
            red_pwd.setRaw(r);
            green_pwd.setRaw(g);
            blue_pwd.setRaw(b);
            return true;
        }

    }

}
