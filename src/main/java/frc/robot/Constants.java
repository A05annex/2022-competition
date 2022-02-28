// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Mk4NeoModule;
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
                // swerve motors and encoders
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
                LF_CALIBRATION = 23,
                // other motors
                FEEDER = 9,
                LIFT_LEFT = 10,
                LIFT_RIGHT = 11,
                COLLECTOR = 12,
                SHOOTER_FRONT = 13,
                SHOOTER_REAR = 14;
    }

    public static final class CalibrationOffset {
        public static final double
                // 2/21 calibration constants: post spin fix
                RF = 2.764,
                RR = 3.559,
                LR = 4.386,
                LF = 4.312;
    }

    // ports for controllers
    public static final int XBOX_PORT = 0;

    // for prototype, length and width from center of the wheels, in m (note chassis is 30" square,
    // the bolt pattern is 29" square, wheels are 2.75" in from the bolt pattern or centered on the
    // corners of a 23.5"(0.5969m) square.
    // For competition, length and width from center of the wheels, in m (note chassis is 28" square,
    // the bolt pattern is 27" square, wheels are 2.75" in from the bolt pattern or centered on the
    // corners of a 21.5"(0.5461m) square.
    public static final double DRIVE_LENGTH = 0.5969;
    public static final double DRIVE_WIDTH = 0.5969;
    public static final double DRIVE_DIAGONAL = Utl.length(DRIVE_LENGTH, DRIVE_WIDTH);

    // drive encoder tics per radian of robot rotation when rotation is controlled by position rather than speed.
    public static final double DRIVE_POS_TICS_PER_RADIAN = 10.385;
    // See the Mk4NeoModule for how this speed is initially estimated.
    public static final double MAX_METERS_PER_SEC = Mk4NeoModule.MAX_METERS_PER_SEC;
    //  The maximum spin of the robot when all that is happening is spin. Since the robot drive centers are
    //  square, att wheels are aligned so their axis passes through the center of that square, and all wheels
    // follow the same circular path at a radius of DRIVE_DIAGONAL/2.0 at MAX_METERS_PER_SEC. So this is
    // computed from DRIVE_DIAGONAL and MAX_METERS_PER_SEC:
    //     Max [radians/sec] = max speed [meters/sec] / (PI * radius) [meters/radian]
    //     Max [radians/sec] = MAX_METERS_PER_SEC / (Math.PI * DRIVE_DIAGONAL * 0.5)
    public static final double MAX_RADIANS_PER_SEC = MAX_METERS_PER_SEC / (Math.PI * DRIVE_DIAGONAL * 0.5);

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
    public static double updateConstant(String key, double initValue) {
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
    public static double updateConstant(String key, double initValue, double lowerBound, double upperBound) {
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
}
