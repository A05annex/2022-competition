// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.a05annex.util.Utl;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class CAN_Devices {
        public static final int
                RF_DRIVE = 1,
                RF_DIRECTION = 2,
                RF_CALIBRATION = 20,
                RR_DRIVE = 3,
                RR_DIRECTION = 4,
                RR_CALIBRATION = 21,
                LR_DRIVE = 5,
                LR_DIRECTION = 6,
                LR_CALIBRATION = 22,
                LF_DRIVE = 7,
                LF_DIRECTION = 8,
                LF_CALIBRATION = 23;
    }

    public static final class CalibrationOffset {
        public static final double
                RF = 2.753,
                RR = 3.562,
                LR = 4.383,
                LF = 4.309;
    }

    // ports for controllers
    public static final int XBOX_PORT = 0;

    // length and width from center of the wheels, in m
    public static final double DRIVE_LENGTH = 0.5842;
    public static final double DRIVE_WIDTH = 0.5842;
    public static final double DRIVE_DIAGONAL = Utl.length(DRIVE_LENGTH, DRIVE_WIDTH);

    // drive encoder tics per radian of robot rotation when rotation is controlled by position rather than speed.
    public static final double DRIVE_POS_TICS_PER_RADIAN = 10.385;
    public static final double MAX_METERS_PER_SEC = 3.2; //TODO: may have changed for the programmer robot w/o weight

    // DriveCommand constants
    // maximum change in joystick value per 20ms for speed and rotation
    public static double DRIVE_MAX_SPEED_INC = 0.075;
    public static double DRIVE_MAX_ROTATE_INC = 0.075;

    // deadband of drive and rotate joysticks
    public static double DRIVE_DEADBAND = 0.05;
    public static double ROTATE_DEADBAND = 0.05;

    // sensitivity and gain
    public static double DRIVE_SPEED_SENSITIVITY = 3.0;
    public static double DRIVE_SPEED_GAIN = 1.0;
    public static double ROTATE_SENSITIVITY = 2.0;
    public static double ROTATE_GAIN = 0.8;

    // kP for keeping drive at the same orientation
    public static double DRIVE_ORIENTATION_kP = 1.2;

    // connect constants to SmartDashboard
    /**
     * Initialize value on SmartDashboard for user input, or if already present, return current value.
     *
     * @param key (String) The key to associate with the value.
     * @param initValue (double) The default value to assign if not already on SmartDashboard.
     *
     * @return The new value that appears on the dashboard.
     */
    public static double updateDriverConstant(String key, double initValue) {
        // if key already exists, value will be the current value or whatever we just typed in to the dashboard
        // if key doesn't exist yet, value will be set to initValue and added to SmartDashboard
        double value = SmartDashboard.getNumber(key, initValue);

        // add number if it doesn't exist, or just set it to its current value
        SmartDashboard.putNumber(key, value);
        return value;
    }

    /**
     * Initialize value on SmartDashboard for user input, or if already present, return current value.
     * If value is outside (lowerBound, upperBound), it will be set to the previous value.
     *
     * @param key (String) The key to associate with the value.
     * @param initValue (double) The default value to assign if not already on SmartDashboard.
     * @param lowerBound (double) Lower bound on the value.
     * @param upperBound (double) Upper bound on the value.
     *
     * @return The new value that appears on the dashboard.
     */
    public static double updateDriverConstant(String key, double initValue, double lowerBound, double upperBound) {
        // if key already exists, value will be the current value or whatever we just typed in to the dashboard
        // if key doesn't exist yet, value will be set to initValue and added to SmartDashboard
        double value = SmartDashboard.getNumber(key, initValue);

        // bounds check
        if (value < lowerBound || value > upperBound) {
            value = initValue;
        }

        // add number if it doesn't exist, or just set it to its current value
        SmartDashboard.putNumber(key, value);
        return value;
    }

    /**
     * Put all driver-specific constants on SmartDashboard, and update them in Constants if a new number was typed in.
     */
    public static void updateAllDriverConstants() {
        DRIVE_DEADBAND = updateDriverConstant("drive deadband", DRIVE_DEADBAND,
                0.0, 1.0);
        DRIVE_SPEED_SENSITIVITY = updateDriverConstant("drive sensitivity", DRIVE_SPEED_SENSITIVITY,
                1.0, Double.MAX_VALUE);
        DRIVE_SPEED_GAIN = updateDriverConstant("drive gain", DRIVE_SPEED_GAIN,
                0.0, Double.MAX_VALUE);

        ROTATE_DEADBAND = updateDriverConstant("rotate deadband", ROTATE_DEADBAND,
                0.0, 1.0);
        ROTATE_SENSITIVITY = updateDriverConstant("rotate sensitivity", ROTATE_SENSITIVITY,
                1.0, Double.MAX_VALUE);
        ROTATE_GAIN = updateDriverConstant("rotate gain", ROTATE_GAIN,
                0.0, Double.MAX_VALUE);
    }

}
