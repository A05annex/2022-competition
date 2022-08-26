package org.a05annex.frc.subsytems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.NavX;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;
import org.a05annex.util.Utl;

public class DriveSubsystem extends SubsystemBase implements ISwerveDrive {
    /**
     * The Singleton instance of this DriveSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private static DriveSubsystem INSTANCE;

    /**
     * Returns the Singleton instance of this DriveSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code DriveSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static DriveSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new DriveSubsystem();
        }
        return INSTANCE;
    }

    // create drive modules
    // first letter is right or left, second is front or rear
    private final Mk4NeoModule m_rf;
    private final Mk4NeoModule m_rr;
    private final Mk4NeoModule m_lf;
    private final Mk4NeoModule m_lr;

    // create NavX - the drive subsystem owns the NavX and is responsible for the heading update
    // cycle.
    private final NavX m_navx = NavX.getInstance();

    // the drive geometry
    private final double LENGTH_OVER_DIAGONAL =
            A05Constants.getDriveLength() / A05Constants.getDriveDiagonal();
    private final double WIDTH_OVER_DIAGONAL =
            A05Constants.getDriveWidth() / A05Constants.getDriveDiagonal();

    // keep track of last angles
    private final AngleD m_RF_lastRadians = new AngleD(AngleD.ZERO);
    private final AngleD m_RR_lastRadians = new AngleD(AngleD.ZERO);
    private final AngleD m_LF_lastRadians = new AngleD(AngleD.ZERO);
    private final AngleD m_LR_lastRadians = new AngleD(AngleD.ZERO);

    // keep track of the last chassis speeds for odometry
    private double m_thisChassisForward = 0.0;
    private double m_thisChassisStrafe = 0.0;
    private long m_lastTime = System.currentTimeMillis();
    private final AngleD m_lastHeading = new AngleD(AngleD.ZERO);
    private double m_lastChassisForward = 0.0;
    private double m_lastChassisStrafe = 0.0;

    private double m_fieldX = 0.0;
    private double m_fieldY = 0.0;
    private final AngleD m_fieldHeading = new AngleD(AngleD.ZERO);

    /**
     * Creates a new instance of this DriveSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private DriveSubsystem() {
        // initialize drive modules
        m_rf = Mk4NeoModule.factory(A05Constants.A05CAN_Devices.RF_DRIVE, A05Constants.A05CAN_Devices.RF_DIRECTION,
                A05Constants.A05CAN_Devices.RF_CALIBRATION, A05Constants.CalibrationOffset.RF);

        m_rr = Mk4NeoModule.factory(A05Constants.A05CAN_Devices.RR_DRIVE, A05Constants.A05CAN_Devices.RR_DIRECTION,
                A05Constants.A05CAN_Devices.RR_CALIBRATION, A05Constants.CalibrationOffset.RR);

        m_lf = Mk4NeoModule.factory(A05Constants.A05CAN_Devices.LF_DRIVE, A05Constants.A05CAN_Devices.LF_DIRECTION,
                A05Constants.A05CAN_Devices.LF_CALIBRATION, A05Constants.CalibrationOffset.LF);

        m_lr = Mk4NeoModule.factory(A05Constants.A05CAN_Devices.LR_DRIVE, A05Constants.A05CAN_Devices.LR_DIRECTION,
                A05Constants.A05CAN_Devices.LR_CALIBRATION, A05Constants.CalibrationOffset.LR);
    }

    // getter methods for modules
    public Mk4NeoModule getRFModule() {
        return m_rf;
    }

    public Mk4NeoModule getRRModule() {
        return m_rr;
    }

    public Mk4NeoModule getLFModule() {
        return m_lf;
    }

    public Mk4NeoModule getLRModule() {
        return m_lr;
    }

    public void resetDrivePID() {
        m_rf.setDrivePID();
        m_rr.setDrivePID();
        m_lf.setDrivePID();
        m_lr.setDrivePID();
    }

    /**
     * Print all module angles to SmartDashboard. Should be called in DriveSubsystem periodic if used.
     */
    public void printAllAngles() {
        SmartDashboard.putNumber("RF cal angle", m_rf.getCalibrationPosition());
        SmartDashboard.putNumber("RR cal angle", m_rr.getCalibrationPosition());
        SmartDashboard.putNumber("LF cal angle", m_lf.getCalibrationPosition());
        SmartDashboard.putNumber("LR cal angle", m_lr.getCalibrationPosition());

        SmartDashboard.putNumber("RF angle", m_rf.getDirectionPosition());
        SmartDashboard.putNumber("RR angle", m_rr.getDirectionPosition());
        SmartDashboard.putNumber("LF angle", m_lf.getDirectionPosition());
        SmartDashboard.putNumber("LR angle", m_lr.getDirectionPosition());
    }

    // begin swerve methods
    /**
     * The internal method to run, or prepare to run, the swerve drive with the specified {@code  forward},
     * {@code strafe}, and {@code rotation} chassis relative components.
     *
     *
     * @param forward  Drive forward. From -1 (full backwards) to 1 (full forwards.
     * @param strafe   Strafe right. From -1 (full left)  to 1 (full right).
     * @param rotation Clockwise rotation. From -1 (full counter-clockwise) to 1 (full clockwise).
     * @param setSpeeds (boolean) {@code true} if module speeds should be set to run the modules, {@code false} if
     *                  this method is being called to prepare (set the direction of) the modules to run this command.
     *                  If {@code false}, module speeds will be 0.0 and there should be no robot motion.
     */
    private void setModulesForChassisMotion(double forward, double strafe,
                                            double rotation, boolean setSpeeds)
    {
        // calculate a, b, c and d variables
        double a = strafe - (rotation * LENGTH_OVER_DIAGONAL);
        double b = strafe + (rotation * LENGTH_OVER_DIAGONAL);
        double c = forward - (rotation * WIDTH_OVER_DIAGONAL);
        double d = forward + (rotation * WIDTH_OVER_DIAGONAL);

        // calculate wheel speeds
        double rfSpeed = Utl.length(b, c);
        double lfSpeed = Utl.length(b, d);
        double lrSpeed = Utl.length(a, d);
        double rrSpeed = Utl.length(a, c);

        // normalize speeds
        double max = Utl.max(rfSpeed, lfSpeed, lrSpeed, rrSpeed);
        if (max > 1.0) {
            rfSpeed /= max;
            lfSpeed /= max;
            lrSpeed /= max;
            rrSpeed /= max;
            forward /= max;
            strafe /= max;
        }

        // if speed is small or 0, (i.e. essentially stopped), use the last angle because its next motion
        // will probably be very close to its current last motion - i.e. the next direction will probably
        // be very close to the last direction.
        double SMALL = 0.000001;
        if (rfSpeed > SMALL) {m_RF_lastRadians.atan2(b, c);}
        if (lfSpeed > SMALL) {m_LF_lastRadians.atan2(b, d);}
        if (lrSpeed > SMALL) {m_LR_lastRadians.atan2(a, d);}
        if (rrSpeed > SMALL) {m_RR_lastRadians.atan2(a, c);}

        // run wheels at speeds and angles
        m_rf.setDirectionAndSpeed(m_RF_lastRadians, setSpeeds ? rfSpeed : 0.0);
        m_lf.setDirectionAndSpeed(m_LF_lastRadians, setSpeeds ? lfSpeed : 0.0);
        m_lr.setDirectionAndSpeed(m_LR_lastRadians, setSpeeds ? lrSpeed : 0.0);
        m_rr.setDirectionAndSpeed(m_RR_lastRadians, setSpeeds ? rrSpeed : 0.0);

        // save the values we set for use in odometry calculations
        m_thisChassisForward = setSpeeds ? forward : 0.0;
        m_thisChassisStrafe = setSpeeds ? strafe : 0.0;
    }

    /**
     * Run the swerve drive with the specified {@code  forward}, {@code strafe}, and {@code rotation} chassis
     * relative components.
     *
     * @param forward  Drive forward. From -1 (full backwards) to 1 (full forwards.
     * @param strafe   Strafe right. From -1 (full left)  to 1 (full right).
     * @param rotation Clockwise rotation. From -1 (full counter-clockwise) to 1 (full clockwise).
     */
    @Override
    public void swerveDriveComponents(double forward, double strafe,
                                      double rotation) {
        setModulesForChassisMotion(forward, strafe, rotation, true);
    }

    /**
     * Prepare the swerve drive to run with the swerve drive with the specified {@code  forward}, {@code strafe},
     * and {@code rotation} chassis relative components. 'Prepare', in this context, means orient all the modules
     * so they are ready to perform this command but set the module speeds to 0.0; In this way movement can start
     * smoothly without additional module reorientation. This method is used to initialize the robot before the
     * start of an autonomous path.
     *
     * @param forward  Drive forward. From -1 (full backwards) to 1 (full forwards.
     * @param strafe   Strafe right. From -1 (full left)  to 1 (full right).
     * @param rotation Clockwise rotation. From -1 (full counter-clockwise) to 1 (full clockwise).
     */
    @Override
    public void prepareForDriveComponents(double forward, double strafe,
                                          double rotation)
    {
        setModulesForChassisMotion(forward, strafe, rotation,false);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            //  If this is interrupted it is because the robot is being shut down - that is OK
        }
    }

    /**
     * Swerve drive with a robot-relative direction, a speed and a rotation speed.
     *
     * @param chassisDirection (AngleConstantD) The robot chassis relative direction in radians from -PI to
     *                         PI where 0.0 is towards the front of the robot, and positive is clockwise.
     * @param speed            (double) Speed from 0.0 to 1.0.
     * @param rotation         (double) Clockwise rotation speed from -1.0 to 1.0.
     */
    @Override
    public void swerveDrive(AngleConstantD chassisDirection, double speed, double rotation) {
        swerveDriveComponents(chassisDirection.cos() * speed,
                chassisDirection.sin() * speed, rotation);
    }

    /**
     * Swerve drive with a field-relative direction, a speed and a rotation.
     *
     * @param fieldDirection (AngleD) The direction in radians from -PI to PI where 0.0 is away from the
     *                       driver, and positive is clockwise.
     * @param speed          (double) Speed from 0.0 to 1.0.
     * @param rotation       (double) Clockwise rotation speed from -1.0 to 1.0.
     */
    @Override
    public void swerveDriveFieldRelative(AngleConstantD fieldDirection, double speed, double rotation) {
        swerveDrive(new AngleD(fieldDirection).subtract(m_navx.getHeading()), speed, rotation);
    }

    //end swerve methods
    //begin odometry methods

    /**
     * Set the field position of the robot. This is typically called at the beginning of the autonomous
     * command as the command that is run should know where the robot has been placed on the field. It
     * could also be called during play if machine vision, sensors, or some other method is available
     * o locate the robot on the field.
     *
     * @param fieldX  (double) The X location of the robot on the field.
     * @param fieldY  (double) The Y location of the robot on the field.
     * @param heading (AngleD) The heading of the robot on the field.
     */
    @Override
    public void setFieldPosition(double fieldX, double fieldY, AngleD heading) {
        m_fieldX = fieldX;
        m_fieldY = fieldY;
        m_fieldHeading.setValue(heading);
        m_navx.initializeHeadingAndNav(m_fieldHeading);
        m_lastTime = System.currentTimeMillis();
    }

    public double getFieldX() {
        return m_fieldX;
    }

    public double getFieldY() {
        return m_fieldY;
    }

    /**
     * Returns the heading of the robot on the field.
     * @return (AngleD) A copy of the heading of the robot.
     */
    public AngleD getFieldHeading() {
        return m_fieldHeading.cloneAngleD();
    }

    /**
     * Rotate the chassis to the specified heading with no field translation. This controls the module using distance
     * (i.e. moving a specified number of ticks) rather than speed because this adjustment of heading is faster
     * and more reliable.
     *
     * @param targetHeading (AngleConstantD) The desired chassis heading on the field.
     */
    public void setHeading(AngleConstantD targetHeading) {
        m_RF_lastRadians.atan2(A05Constants.getDriveLength(), -A05Constants.getDriveWidth());
        m_LF_lastRadians.atan2(A05Constants.getDriveLength(), A05Constants.getDriveWidth());
        m_LR_lastRadians.atan2(-A05Constants.getDriveLength(), A05Constants.getDriveWidth());
        m_RR_lastRadians.atan2(-A05Constants.getDriveLength(), -A05Constants.getDriveWidth());

        double deltaTics = new AngleD(targetHeading).subtract(m_navx.getHeading()).getRadians()
                * A05Constants.getDrivePosTicsPerRadian();

        m_rf.setDirectionAndDistance(m_RF_lastRadians, deltaTics);
        m_lf.setDirectionAndDistance(m_LF_lastRadians, deltaTics);
        m_lr.setDirectionAndDistance(m_LR_lastRadians, deltaTics);
        m_rr.setDirectionAndDistance(m_RR_lastRadians, deltaTics);

        m_thisChassisForward = 0.0;
        m_thisChassisStrafe = 0.0;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Update the NavX heading
        m_navx.recomputeHeading(false);
        // Update the odometry for the drive. OK, the scam here is that there was a previous heading set
        // in the last command cycle when we were setting the new direction/speed/rotation for the
        // chassis, and the heading we are at now. For odometry, assume the average of the last heading and current
        // heading approximates the path of the robot and that the last speed set happened pretty
        // instantaneously. In that case, we can make a pretty good guess how the robot moved on the field.

        // Get the average speed and heading for this interval
        AngleD currentHeading = m_navx.getHeading();
        AngleD aveHeading = currentHeading.cloneAngleD().add(m_lastHeading).mult(0.5);
        double aveForward = (m_lastChassisForward + m_thisChassisForward) * 0.5;
        double aveStrafe = (m_lastChassisStrafe + m_thisChassisStrafe) * 0.5;

        // the maximum distance we could travel in this interval at max speed
        long now = System.currentTimeMillis();
        double maxDistanceInInterval = A05Constants.getMaxMetersPerSec() * (double) (now - m_lastTime) / 1000.0;

        // compute the distance in field X and Y and update the field position
        double sinHeading = aveHeading.sin();
        double cosHeading = aveHeading.cos();
        m_fieldX += ((aveForward * sinHeading) + (aveStrafe * cosHeading)) * maxDistanceInInterval;
        m_fieldY += ((aveForward * cosHeading) - (aveStrafe * sinHeading)) * maxDistanceInInterval;

        // save the current state as the last state
        m_lastHeading.setValue(currentHeading);
        m_fieldHeading.setValue(currentHeading);
        m_lastChassisForward = m_thisChassisForward;
        m_lastChassisStrafe = m_thisChassisStrafe;
        m_lastTime = now;

//        // telemetry
//        printAllAngles();
    }
}

