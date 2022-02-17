package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;


/**
 * This is a class that initializes and tracks the NavX board to maintain current information, specifically
 * heading, for the robot. We have been having a degrees vs. radians, since all of the math trig libraries
 * use radians. we decided that staying in radians would build up minimal round-off error.
 *
 * Originally this class was written to support NavX on a conventional drive that had PID direction loops
 * concerned with matching actual heading to expected heading. Right now we are a little unclear how that
 * relates to the A05annex 2021 season swerve drive.
 */
public class NavX {

    //==================================================================================================================
    // NOTE: the NavX software expresses all the navigation angles in degrees, so we maintain angles internal
    // to this class in degrees. And do the conversions to radians when this class is queried for values.
    private final AHRS m_ahrs;
    /** The heading we are trying to track with the robot.
     */
    private final AngleD m_expectedHeading = new AngleD(AngleD.ZERO);
    private double m_updateCt = -1;

    /** The raw heading, not corrected for the spins, read directly from the NavX, in the range
     * -180 to +180 degrees. Used for determining whether the boundary between -180 and 180 has been crossed.
     */
    private final AngleD m_headingRawLast = new AngleD(AngleD.ZERO);

    /** The number of complete revolutions the robot has made.
     */
    private int m_headingRevs = 0;

    /** The actual heading of the robot from -infinity to infinity, so the spins are included in this
     *  heading.
     */
    private final AngleD m_heading = new AngleD(AngleD.ZERO);
    private boolean m_setExpectedToCurrent = false;

    // --------------------------------------------------
    // Reference values - these are the values at initialization of the NavX recording the position of the robot
    // on the field in terms of the readings of the NavX and the field heading of the robot at initialization. We
    // refer to these as the reference values as everything that happens after initialization is evaluated
    // with reference to (or as the change relative to this reference).
    // --------------------------------------------------
    /**
     * The NavX reported pitch at the time the NavX in initialized.
     */
    private final AngleD m_refPitch = new AngleD(AngleD.ZERO);
    /**
     * The NavX reported yaw at the time the NavX in initialized.
     */
    private final AngleD m_refYaw = new AngleD(AngleD.ZERO);
    /**
     * The NavX reported roll at the time the NavX in initialized.
     */
    private final AngleD m_refRoll = new AngleD(AngleD.ZERO);
    /**
     * The actual field heading of the robot at the time the NavX in initialized.
     */
    private final AngleD m_refHeading = new AngleD(AngleD.ZERO);

    /**
     *
     */
    private NavX() {
        // So, if there is no navx, there is no error - it just keeps trying to connect forever, so this
        // needs to be on a thread that can be killed if it doesn't connect in time ......
        // TODO: figure out the threading, error handling, and redundancy.
        m_ahrs = new AHRS(SPI.Port.kMXP);
        m_ahrs.reset();
        while (m_ahrs.isCalibrating()) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                break;
            }
        }
        m_updateCt = m_ahrs.getUpdateCount();
        initializeHeadingAndNav();
    }

    /**
     * Sets the reference start heading and navigation reference positions to the current values. This should
     * be called immediately at the start of autonomous.
     */
    public void initializeHeadingAndNav() {
        initializeHeadingAndNav(AngleD.ZERO);
    }

    /**
     * Sets the reference start heading and navigation reference positions to the current values. This should
     * be called immediately at the start of autonomous.
     *
     * @param heading (AngleConstantD) The current field heading of the robot.
     */
    public void initializeHeadingAndNav(AngleConstantD heading) {
        // In the past we have always initialized with the front of the robot facing down field, so the
        // heading was 0.0 at initialization. In this case we are
        m_refPitch.setDegrees(m_ahrs.getPitch());
        m_refYaw.setDegrees(m_ahrs.getYaw());
        m_refRoll.setDegrees(m_ahrs.getRoll());
        m_refHeading.setValue(heading);
        m_headingRawLast.setValue(AngleD.ZERO);
        m_expectedHeading.setValue(m_refHeading);
        m_headingRevs = 0;
    }

    /**
     * Change the expected heading by the specified angle.
     *
     * @param delta The change to the expected heading.
     */
    public void incrementExpectedHeading(AngleD delta) {
        m_expectedHeading.add(delta);

    }

    /**
     * Set the expected heading to the current heading.
     */
    public void setExpectedHeadingToCurrent() {
        m_expectedHeading.setValue(m_heading);
    }

    /**
     * Recompute the heading as reported by the NavX and adjusted to be always increasing when rotation is
     * clockwise. This heading computation was introduced by Jason Barringer to the FRC 6831 AO5 Annex code base
     * in the 2017 season to make using PID loops to control heading with the IMU easier to write, and more
     * predictable. If there is a discontinuity in the sensor output, this means there needs to be special logic
     * in the PID code to deal with the discontinuity. This handles the discontinuity in a single place where
     * the heading is computed.
     *
     * @param setExpectedToCurrent (boolean) {@code true} if the expected heading should be set to the current
     *                             heading, {@code false} otherwise. This would normally be {@code true} during
     *                             robot-relative driving when the driver is turning (the expected heading is
     *                             where the driver is turning to). This would normally be {@code false} during
     *                             field-relative driving or autonomous when the program is setting a target
     *                             heading and the robot is the expected to move along, or turn towards, the
     *                             expected heading; or when robot-relative driving without any turn.
     */
    public void recomputeHeading(boolean setExpectedToCurrent) {
        m_setExpectedToCurrent = setExpectedToCurrent;
        AngleD heading_raw = new AngleD(AngleUnit.DEGREES, m_ahrs.getYaw());
        // This is the logic for detecting and correcting for the IMU discontinuity at +180degrees and -180degrees.
        if (m_headingRawLast.isLessThan(AngleD.NEG_PI_OVER_2) && heading_raw.isGreaterThan(AngleD.ZERO)) {
            // The previous raw IMU heading was negative and close to the discontinuity, and it is now positive. We
            // have gone through the discontinuity so we decrement the heading revolutions by 1 (we completed a
            // negative revolution). NOTE: the initial check protects from the case that the heading is near 0 and
            // goes continuously through 0, which is not the completion of a revolution.
            m_headingRevs--;
        } else if (m_headingRawLast.isGreaterThan(AngleD.PI_OVER_2) && heading_raw.isLessThan(AngleD.ZERO)) {
            // The previous raw IMU heading was positive and close to the discontinuity, and it is now negative. We
            // have gone through the discontinuity so we increment the heading revolutions by 1 (we completed
            // positive revolution). NOTE: the initial check protects from the case that the heading is near 0 and
            // goes continuously through 0, which is not the completion of a revolution.
            m_headingRevs++;
        }
        m_headingRawLast.setValue(heading_raw);

        m_heading.setRadians(m_headingRevs * AngleD.TWO_PI.getRadians())
                .add(heading_raw).subtract(m_refYaw).add(m_refHeading);

        if (setExpectedToCurrent) {
            m_expectedHeading.setValue(m_heading);
        }
    }

    /**
     * Returns a copy of the robot chassis heading.
     *
     * @return A copy of the robot chassis heading.
     */
    public AngleD getHeading() {
        return m_heading.cloneAngleD();
    }

    /**
     * @return Returns the heading info, returns {@code null} if there is a problem with the NavX.
     */
    public HeadingInfo getHeadingInfo() {
        if (null == m_ahrs) {
            return null;
        }
        double updateCt = m_ahrs.getUpdateCount();
        if (updateCt <= m_updateCt) {
            // there is a problem communication with the NavX - the results we would get from NavX queries
            // are unreliable.
            return null;
        }
        return new HeadingInfo(m_heading, m_expectedHeading, m_setExpectedToCurrent);
    }

    /**
     * @return Returns the navigation info, returns {@code null} if there is a problem with the NavX.
     */
    public NavInfo getNavInfo() {
        if (null == m_ahrs) {
            return null;
        }
        // The subtraction of the ref values adjusts for the construction bias of not having the NavX perfectly
        // mounted, or there being some bias in the NavX - i.e. the ref represents the value first reported when
        // the reference position is set, see initializeHeadingAndNav().
        return new NavInfo(
                new AngleConstantD(AngleUnit.DEGREES, m_ahrs.getPitch() - m_refPitch.getDegrees()),
                new AngleConstantD(AngleUnit.DEGREES, m_ahrs.getYaw() - m_refYaw.getDegrees()),
                new AngleConstantD(AngleUnit.DEGREES, m_ahrs.getRoll() - m_refRoll.getDegrees()),
                new AngleConstantD(AngleUnit.DEGREES, m_ahrs.getPitch()),
                new AngleConstantD(AngleUnit.DEGREES, m_ahrs.getYaw()),
                new AngleConstantD(AngleUnit.DEGREES, m_ahrs.getRoll()));
    }

    public static class HeadingInfo {
        /**
         * The current heading in radians of the robot as computed in the last call
         * to {@link NavX#initializeHeadingAndNav()}.
         */
        public final AngleConstantD heading;
        public final AngleConstantD expectedHeading;
        public final boolean isExpectedTrackingCurrent;

        HeadingInfo(AngleD heading, AngleD expectedHeading, boolean isExpectedTrackingCurrent) {
            this.heading = heading;
            this.expectedHeading = expectedHeading;
            this.isExpectedTrackingCurrent = isExpectedTrackingCurrent;
        }
    }

    /**
     * The data class for the 'raw' navigation info from the NavX, corrected by when the reference was last set.
     */
    public static class NavInfo {
        /**
         * The pitch (lean forward or backward) of the robot, with negative being forwards, from when the robot
         * was first initialized.
         */
        public final AngleConstantD pitch;
        public final AngleConstantD rawPitch;
        /**
         * The yaw (rotation or turn) of the robot, with positive being clockwise (to the right), from when the
         * robot was first initialized.
         */
        public final AngleConstantD yaw;
        public final AngleConstantD rawYaw;
        /**
         * The roll (lean sideways) of the robot, with positive being the robot falling over on it's left
         * side, from when the robot was first initialized.
         */
        public final AngleConstantD roll;
        public final AngleConstantD rawRoll;

        NavInfo(AngleConstantD pitch, AngleConstantD yaw, AngleConstantD roll,
                AngleConstantD rawPitch, AngleConstantD rawYaw, AngleConstantD rawRoll) {
            this.pitch = pitch;
            this.yaw = yaw;
            this.roll = roll;
            this.rawPitch = rawPitch;
            this.rawYaw = rawYaw;
            this.rawRoll = rawRoll;
        }
    }

    /**
     * The Singleton instance of this NavX. External classes should
     * use the {@link #getInstance()} method to get the instance.
     */
    private final static NavX INSTANCE = new NavX();

    /**
     * Returns the Singleton instance of this NavX. This static method
     * should be used -- {@code NavX.getInstance();} -- by external
     * classes, rather than the constructor to get the instance of this class.
     */
    public static NavX getInstance() {
        return INSTANCE;
    }
}
