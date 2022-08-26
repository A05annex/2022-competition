package org.a05annex.frc;

import frc.robot.Constants;
import org.a05annex.frc.subsytems.Mk4NeoModule;
import org.a05annex.util.Utl;

public abstract class A05Constants {

    public static class A05CAN_Devices {
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
                LF_CALIBRATION = 23;
    }

    public static final class CalibrationOffset {
        public static double RF, RR, LF, LR;
    }

    private static double DRIVE_LENGTH, DRIVE_WIDTH, DRIVE_DIAGONAL;

    // See the Mk4NeoModule for how this speed is initially estimated.
    //    public static final double MAX_METERS_PER_SEC = Mk4NeoModule.MAX_METERS_PER_SEC * (190.0/197.0); // tested
    private static double MAX_METERS_PER_SEC = Mk4NeoModule.MAX_METERS_PER_SEC;
    //  The maximum spin of the robot when all that is happening is spin. Since the robot drive centers are
    //  square, att wheels are aligned so their axis passes through the center of that square, and all wheels
    // follow the same circular path at a radius of DRIVE_DIAGONAL/2.0 at MAX_METERS_PER_SEC. So this is
    // computed from DRIVE_DIAGONAL and MAX_METERS_PER_SEC:
    //     Max [radians/sec] = max speed [meters/sec] / (PI * radius) [meters/radian]
    //     Max [radians/sec] = MAX_METERS_PER_SEC / (Math.PI * DRIVE_DIAGONAL * 0.5)
    private static double MAX_RADIANS_PER_SEC;

    // drive encoder tics per radian of robot rotation when rotation is controlled by position rather than speed.
    public static double DRIVE_POS_TICS_PER_RADIAN;

    public static double DRIVE_ORIENTATION_kP;

    public static double getDriveLength() {
        return DRIVE_LENGTH;
    }

    public static double getDriveWidth() {
        return DRIVE_WIDTH;
    }

    public static double getDriveDiagonal() {
        return DRIVE_DIAGONAL;
    }

    public static double getMaxMetersPerSec() {
        return MAX_METERS_PER_SEC;
    }

    public static double getMaxRadiansPerSec() {
        return MAX_RADIANS_PER_SEC;
    }

    public static double getDrivePosTicsPerRadian() {
        return DRIVE_POS_TICS_PER_RADIAN;
    }

    public static double getDriveOrientationkp() {
        return DRIVE_ORIENTATION_kP;
    }

    public static void setDriveGeometry(double length, double width) {
        DRIVE_LENGTH = length;
        DRIVE_WIDTH = width;
        DRIVE_DIAGONAL = Utl.length(DRIVE_LENGTH, DRIVE_WIDTH);
        MAX_RADIANS_PER_SEC = (377.0/360.0) * // tested
                ((MAX_METERS_PER_SEC * 2 * Math.PI) / (Math.PI * DRIVE_DIAGONAL));
        DRIVE_POS_TICS_PER_RADIAN = 10; //TODO: Compute this number
    }

    public static void setDriveOrientationkp(double kp) {
        DRIVE_ORIENTATION_kP = kp;
    }

    public static void setDriveCalibration(double rf, double rr, double lf, double lr) {
        CalibrationOffset.RF = rf;
        CalibrationOffset.RR = rr;
        CalibrationOffset.LF = lf;
        CalibrationOffset.LR = lr;
    }
}
