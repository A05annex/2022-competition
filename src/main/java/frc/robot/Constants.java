// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.a05annex.frc.A05Constants;
import org.a05annex.util.geo2d.KochanekBartelsSpline;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants extends A05Constants {

    public static final class CAN_Devices {
        public static final int
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
//                RF = 2.764,
//                RR = 3.559,
//                LR = 4.386,
//                LF = 4.312;
                // Programming base
                  RF = 0.785,
                  RR = 0.357,
                  LR = 2.519,
                  LF = 0.563;
    }

    public static final class LimelightCalibrationPoint {
        public final double ty;
        public final double frontSpeed;
        public final double rearSpeed;
        public final double xOffset;

        public LimelightCalibrationPoint(double ty, double frontSpeed, double rearSpeed, double xOffset) {
            this.ty = ty;
            this.frontSpeed = frontSpeed;
            this.rearSpeed = rearSpeed;
            this.xOffset = xOffset;
        }
    }

    public static final LimelightCalibrationPoint[] LIMELIGHT_CALIBRATION_POINTS = {
        new LimelightCalibrationPoint(1.9, 0.5, -0.55, 4.20),
        new LimelightCalibrationPoint(-0.9, 0.55, -0.55, 4.20),
        new LimelightCalibrationPoint(-5.2, 0.6, -0.55, 5.20),
        new LimelightCalibrationPoint(-7.7, 0.725, -0.625, 4.20)
    };

    // ports for controllers
    public static final int XBOX_PORT = 0;
    public static final int HANG_XBOX_PORT = 1;

    // limelight and field position for distance calculation
    public static final double TARGET_HEIGHT = 2.62; // meters
    public static final double TARGET_RADIUS = 0.61; // meters
    public static final double LIMELIGHT_HEIGHT = 0.99; // meters
    public static final double LIMELIGHT_ANGLE = 64.0; // degrees

    // for prototype, length and width from center of the wheels, in m (note chassis is 30" square,
    // the bolt pattern is 29" square, wheels are 2.75" in from the bolt pattern or centered on the
    // corners of a 23.5"(0.5969m) square.
    // For competition, length and width from center of the wheels, in m (note chassis is 28" square,
    // the bolt pattern is 27" square, wheels are 2.75" in from the bolt pattern or centered on the
    // corners of a 21.5"(0.5461m) square.
    public static final double DRIVE_LENGTH = 0.5969;
    public static final double DRIVE_WIDTH = 0.5969;

    // kP for keeping drive at the same orientation
    public static double DRIVE_ORIENTATION_kP = 0.3;

    // Digital input switchboard
    private static final DigitalInput switch0 = new DigitalInput(4);
    private static final DigitalInput switch1 = new DigitalInput(3);
    private static final DigitalInput switch2 = new DigitalInput(2);
    private static final DigitalInput switch3 = new DigitalInput(1);
    private static final DigitalInput switch4 = new DigitalInput(0);

    public static int readDriverID() {
        return (switch0.get() ? 0 : 1) + (switch1.get() ? 0 : 2);
    }

    public static int readAutoID() {
        return (switch2.get() ? 0 : 1) + (switch3.get() ? 0 : 2) + (switch4.get() ? 0 : 4);
    }

    public static void printIDs() {
        SmartDashboard.putNumber("driver", readDriverID());
        SmartDashboard.putNumber("auto", readAutoID());
    }

    // amount of time to wait in auto
    public static int AUTO_WAIT_CYCLES = 50;

    // enum that contains autos
    public enum AutonomousPath {
        FOUR_BALL("4Ball", 0, "2022_4ball.json"),
        TWO_BALL_A("2Ball A", 1, "2022_2ball_A.json"),
        TWO_BALL_B("2Ball B", 2, "2022_2ball_B.json"),
        TWO_BALL_C("2Ball C", 3, "2022_2ball_C.json"),
        THREE_BALL_WESTVALLEY("3Ball West Valley", 4, "2022_3ball_westvalley.json");

        static AutonomousPath AUTONOMOUS_PATH = AutonomousPath.TWO_BALL_B;

        private final String m_pathName;
        private final int m_id;
        private final String m_filename;

        AutonomousPath(String skill, int id, String filename) {
            m_pathName = skill;
            m_id = id;
            m_filename = filename;
        }

        public static String getName() {
            return AUTONOMOUS_PATH.m_pathName;
        }

        /**
         * Load this autonomous path.
         *
         * @return The loaded path, {@code null} if the path could not be loaded.
         */
        public static KochanekBartelsSpline load() {
            KochanekBartelsSpline spline = new KochanekBartelsSpline();
            if (spline.loadPath(Filesystem.getDeployDirectory() + "/paths/" +
                    AUTONOMOUS_PATH.m_filename)) {
                return spline;
            } else {
                return null;
            }
        }

        public static void setAutonomousToId(int id) {
            AUTONOMOUS_PATH = AutonomousPath.TWO_BALL_A;
            for (AutonomousPath path : values()) {
                if (path.m_id == id) {
                    AUTONOMOUS_PATH = path;
                    break;
                }
            }
        }
    }

    // kP for keeping target orientation while driving
    public static final double TARGET_kP = 0.5;

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
