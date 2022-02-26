package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;
import org.jetbrains.annotations.NotNull;

/**
 * This class represents and controls an
 * <a href="https://www.swervedrivespecialties.com/collections/kits/products/mk4-swerve-module">SDS MK4</a>
 * Swerve Drive module powered by <a href="https://www.revrobotics.com/rev-21-1650/">REV Neo</a> motors controlled
 * using <a href="https://www.revrobotics.com/rev-11-2158/">REV Spark Max</a> motor controllers and with a
 * <a href="https://store.ctr-electronics.com/cancoder/">CTRE CANcoder</a> as the position encoder for
 * absolute wheel direction.
 */
public class Mk4NeoModule {

    // -----------------------------------------------------------------------------------------------------------------
    // The control constants for this specific serve module (The MK4 with Neo motors, Spark Max controllers,
    // and a CANcoder calibration encoder) with the standard gear ratio 8:14 to 1, and a Neo unadjusted free
    // speed of 12.0ft/sec. We measured the unloaded max speed of the Neo at about 5700RPM, here we limit the
    // maximum speed to 5000rpn, which is .8772 of the maximum free speed (which is specified as 12 ft/sec), or
    // 10.5ft/sec (3.2m/sec)
    // -----------------------------------------------------------------------------------------------------------------
    /**
     * The maximum speed (RPM) we will ever request from the drive motor. The actual not load maximum RPM with
     * fully charged batteries is about 5700RPM, so this gives about 12% headroom for friction, drag, battery
     * depletion, etc.
     */
    static final double MAX_DRIVE_RPM = 5000;
    /**
     * The maximum speed of the module with this module implementation.
     */
    public static final double MAX_METERS_PER_SEC = 3.2;
    /**
     * Based on telemetry feedback, 1 wheel direction revolution maps to 18 spin encoder revolutions
     */
    static final double RADIANS_TO_SPIN_ENCODER = 12.7999 / AngleD.TWO_PI.getRadians();

    // PID values for the spin spark motor encoder position controller PID loop
    static double SPIN_kP = 0.5;
    static double SPIN_kI = 0.0;

    // PID values for the drive spark motor controller speed PID loop
    static double DRIVE_kP = 0.00003;
    static double DRIVE_kI = 0.000002;
    static double DRIVE_kFF = 0.000174;
    static double DRIVE_IZONE = 200.0;

    // PID values for the drive spark motor controller position PID loop
    static double DRIVE_POS_kP = 0.13;
    static double DRIVE_POS_kI = 0.0;

    // -----------------------------------------------------------------------------------------------------------------
    // The module physical hardware
    // -----------------------------------------------------------------------------------------------------------------
    // This is the physical hardware wired to the roborio
    private final CANSparkMax driveMotor;
    private final CANSparkMax directionMotor;
    private final CANCoder calibrationEncoder;

    // These are the components of the physical hardware
    private final RelativeEncoder driveEncoder;
    private final SparkMaxPIDController drivePID;
    private final RelativeEncoder directionEncoder;
    private final SparkMaxPIDController directionPID;

    // -----------------------------------------------------------------------------------------------------------------
    // The module physical state
    // -----------------------------------------------------------------------------------------------------------------
    // This is the initial 0.0 degree position calibration
    private final double calibrationOffset;
    /**
     * A multiplier for the speed that is either 1.0 (forward) or -1.0 (backwards) because the shortest
     * spin to the desired direction may be the backwards direction of the wheel, which requires the speed
     * to be reversed.
     */
    private double speedMultiplier = 1.0;
    /**
     * The last angle the wheel was set to, in radians. this may be either the front or the back of
     * the wheel - see {@link #speedMultiplier ) documentation for determining whether this is the
     * orientation of the front or the back. This will be in the range -pi to pi.
     */
    private AngleD lastDirection = new AngleD(AngleUnit.RADIANS, 0.0);
    /**
     * The last direction encoder value that was set. Note, we always set the next spin by using a change angle.
     * This means the encoder setting can be anywhere from -infinity to +infinity.
     */
    private double lastDirectionEncoder = 0.0;
    /**
     * The last speed value that was set for this module, in the range 0.0 to 1.0.
     */
    private double lastSpeed = 0.0;

    /**
     * Whether the last command was drive by speed, <code>true</code>, or drive by distance, <code>false</code>.
     */
    private boolean driveBySpeed = true;

    /**
     * * The factory that creates the DriveModule given the
     *
     * @param driveCAN          CAN address for the motor that drives the wheel forward.
     * @param spinCAN           CAN address for the motor that spins the wheel around.
     * @param calibrationCAN    CAN address for the CANcoder.
     * @param calibrationOffset The value of the direction potentiometer that will point the module forward.
     * @return (not null) Returns the initialized drive module.
     */
    public static Mk4NeoModule factory(int driveCAN, int spinCAN, int calibrationCAN, double calibrationOffset) {
        // basic code representations for physical hardware
        CANSparkMax driveMotor = new CANSparkMax(driveCAN, MotorType.kBrushless);
        CANSparkMax spinMotor = new CANSparkMax(spinCAN, MotorType.kBrushless);
        CANCoder calibrationEncoder = new CANCoder(calibrationCAN);
        // derived representations of components embedded in the physical hardware
        RelativeEncoder driveEncoder = driveMotor.getEncoder();
        SparkMaxPIDController drivePID = driveMotor.getPIDController();
        RelativeEncoder spinEncoder = spinMotor.getEncoder();
        SparkMaxPIDController spinPID = spinMotor.getPIDController();
        return new Mk4NeoModule(driveMotor, driveEncoder, drivePID,
                spinMotor, spinEncoder, spinPID,
                calibrationEncoder, calibrationOffset);
    }

    /**
     * Instantiate a DriveModule. All of instanced robot hardware control classes are passed in so this
     * module can be tested using the JUnit test framework.
     *
     * @param driveMotor         (CANSparkMax, not null) The drive motor controller.
     * @param driveEncoder       (RelativeEncoder, not null) The drive motor encoder.
     * @param drivePID           (CANPIDController, not null) The drive motor PID controller.
     * @param directionMotor     (CANSparkMax, not null) The spin motor controller.
     * @param directionEncoder   (RelativeEncoder, not null) The spin motor encoder.
     * @param directionPID       (CANPIDController, not null) The spin motor PID controller.
     * @param calibrationEncoder (CANCoder, not null) The spin analog position encoder which provides
     *                           the absolute spin position of the module.
     * @param calibrationOffset  The value of the analog potentiometer that will point the module forward.
     */
    public Mk4NeoModule(@NotNull CANSparkMax driveMotor, @NotNull RelativeEncoder driveEncoder,
                        @NotNull SparkMaxPIDController drivePID, @NotNull CANSparkMax directionMotor,
                        @NotNull RelativeEncoder directionEncoder, @NotNull SparkMaxPIDController directionPID,
                        @NotNull CANCoder calibrationEncoder, double calibrationOffset) {

        this.driveMotor = driveMotor;
        this.driveEncoder = driveEncoder;
        this.drivePID = drivePID;
        this.directionMotor = directionMotor;
        this.directionEncoder = directionEncoder;
        this.directionPID = directionPID;
        this.calibrationEncoder = calibrationEncoder;

        // Initialize the calibration CANcoder
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.sensorCoefficient = 2 * Math.PI / 4096.0;
        config.unitString = "rad";
        config.sensorDirection = true;
        calibrationEncoder.configAllSettings(config);

        // reset motor controllers to factory default
        this.driveMotor.restoreFactoryDefaults();
        this.directionMotor.restoreFactoryDefaults();

        // invert the spin so positive is a clockwise spin
        this.directionMotor.setInverted(true);

        // update PID controllers for spin and drive motors and initialize them
        initPID(this.drivePID, DRIVE_kFF, DRIVE_kP, DRIVE_kI, DRIVE_IZONE);
        initPID(this.directionPID, 0.0, SPIN_kP, SPIN_kI, 0.0);

        // calibrate
        this.calibrationOffset = calibrationOffset;
        calibrate(); // reset direction encoder position
        this.directionPID.setReference(0.0, CANSparkMax.ControlType.kPosition);
        lastDirection.setValue(AngleUnit.RADIANS, 0.0);
        lastDirectionEncoder = 0.0;
    }

    /**
     * Updates the spin CANPIDController object using values in constants file. Used only when tuning the PID
     * constants for best control.
     */
    public void setSpinPID() {
        directionPID.setP(SPIN_kP);
        directionPID.setI(SPIN_kI);
    }

    /**
     * Updates the drive CANPIDController object using values in constants file. Used only when tuning the PID
     * * constants for best control.
     */
    public void setDrivePID() {
        drivePID.setP(DRIVE_kP);
        drivePID.setI(DRIVE_kI);
        drivePID.setFF(DRIVE_kFF);
        drivePID.setIZone(DRIVE_IZONE);
    }

    public void setDrivePosPID() {
        drivePID.setP(DRIVE_POS_kP);
        drivePID.setI(DRIVE_POS_kI);
        drivePID.setFF(0.0);
        drivePID.setIZone(0.0);
    }

    private void initPID(SparkMaxPIDController pid, double kFF, double kP, double kI, double kIZone) {
        pid.setFF(kFF);
        pid.setP(kP);
        pid.setI(kI);
        pid.setD(0.0);
        pid.setIZone(kIZone);
        pid.setOutputRange(-1.0, 1.0);
    }

    /**
     * Returns the drive motor velocity (RPM) as read from the encoder
     *
     * @return The drive motor velocity (RPM)
     */
    public double getDriveEncoderVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Returns the drive motor position as read from the encoder.
     *
     * @return The drive motor position as read from the encoder.
     */
    public double getDriveEncoderPosition() {
        return driveEncoder.getPosition();
    }

    /**
     * Returns the direction motor position as read from the encoder.
     *
     * @return The direction motor position as read from the encoder.
     */
    public double getDirectionPosition() {
        return directionEncoder.getPosition();
    }

    /**
     * Returns the value of the calibration encoder as a double. The value goes from 0.0 to 2pi, wrapping
     * around when the boundary between 0 and 2pi is reached. This method is provided primarily to read
     * the calibration encoder to determine the calibrationOffset that should be used for initialization.
     *
     * @return The analog direction encoder position.
     */
    public double getCalibrationPosition() {
        return calibrationEncoder.getAbsolutePosition();
    }

    /**
     * Returns the last speed that was set for this module in m/sec.
     *
     * @return The last speed that was set in m/sec.
     */
    public double getLastSpeed() {
        return lastSpeed * MAX_DRIVE_RPM;
    }

    /**
     * Returns the last speed that was set for this module normalized to 0.0-1.0.
     *
     * @return the last normalized speed that was set.
     */
    public double getLastNormalizedSpeed() {
        return lastSpeed;
    }

    /**
     * Get the last direction set for the module.
     *
     * @return the last direction set.
     */
    public AngleD getLastDirection() {
        return lastDirection;
    }

    /**
     * Set the NEO direction encoder value using the absolute direction encoder, so that forward is an encoder
     * reading of 0 tics.
     */
    private void calibrate() {
        // (actual - offset) * 360 / 20
        double absolutePosition;
        do {
            absolutePosition = calibrationEncoder.getAbsolutePosition();
        } while (absolutePosition < 0.0 || absolutePosition > Math.PI*2);
        directionEncoder.setPosition(
                (calibrationEncoder.getAbsolutePosition() - calibrationOffset) * RADIANS_TO_SPIN_ENCODER);
    }

//    /**
//     * Set the direction and speed of the drive wheel in this module.
//     *
//     * @param targetDegrees (double) The direction from -180.0 to 180.0 degrees where 0.0 is towards the
//     *                      front of the robot, and positive is clockwise.
//     * @param speed         (double) The normalized speed of the wheel from 0.0 to 1.0 where 1.0 is the maximum
//     *                      forward velocity.
//     */
//    public void setDegreesAndSpeed(double targetDegrees, double speed) {
//        setRadiansAndSpeed(Math.toRadians(targetDegrees), speed);
//    }

    /**
     * Set the module direction in radians. This code finds the closest forward-backward direction and sets the
     * foward-backaward multiplier for speed.
     *
     * @param targetDirection (AngleD) The direction from -pi to pi radians where 0.0 is towards the
     *                        front of the robot, and positive is clockwise.
     */
    public void setDirection(AngleConstantD targetDirection) {
        // The real angle of the front of the wheel is 180 degrees away from the current angle if the wheel
        // is going backwards (i.e. the lastDirection was the last target angle for the module
        AngleD realLastDirection = (speedMultiplier > 0.0) ? lastDirection :
                lastDirection.isLessThan(AngleD.ZERO) ? lastDirection.add(AngleD.PI) : lastDirection.subtract(AngleD.PI);
        AngleD deltaDirection = new AngleD(targetDirection).subtract(realLastDirection);
        speedMultiplier = 1.0;

        // Since there is wrap-around at -180.0 and 180.0, it is easy to create cases where only a small correction
        // is required, but a very large deltaDegrees results because the spin is in the wrong direction. If the
        // angle is greater than 180 degrees in either direction, the spin is the wrong way. So the next section
        // checks that and changes the direction of the spin is the wrong way.
        if (deltaDirection.isGreaterThan(AngleD.PI)) {
            deltaDirection.subtract(AngleD.TWO_PI);
        } else if (deltaDirection.isLessThan(AngleD.NEG_PI)) {
            deltaDirection.add(AngleD.TWO_PI);
        }

        // So, the next bit is looking at whether it better to spin the front of the wheel to the
        // target and drive forward, or, to spin the back of the wheel to the target direction and drive
        // backwards - if the spin is greater than 90 degrees (pi/2 radians) either direction, it is better
        // to spin the shorter angle and run backwards.
        if (deltaDirection.isGreaterThan(AngleD.PI_OVER_2)) {
            deltaDirection.subtract(AngleD.PI);
            speedMultiplier = -1.0;
        } else if (deltaDirection.isLessThan(AngleD.NEG_PI_OVER_2)) {
            deltaDirection.add(AngleD.PI);
            speedMultiplier = -1.0;
        }

        // Compute and set the spin value
        lastDirection.setValue(targetDirection);
        lastDirectionEncoder += (deltaDirection.getRadians() * RADIANS_TO_SPIN_ENCODER);

        directionPID.setReference(lastDirectionEncoder, CANSparkMax.ControlType.kPosition);
    }

    /**
     * Set the direction and speed of the drive wheel in this module.
     *
     * @param targetDirection (double) The direction from -pi to pi radians where 0.0 is towards the
     *                        front of the robot, and positive is clockwise.
     * @param speed           (double) The normalized speed of the wheel from 0.0 to 1.0 where 1.0 is the maximum
     *                        forward velocity.
     */
    public void setDirectionAndSpeed(AngleConstantD targetDirection, double speed) {

        setDirection(targetDirection);

        // Compute and set the speed value
        lastSpeed = speed;
        speed *= MAX_DRIVE_RPM * speedMultiplier;

        if (!driveBySpeed) {
            setDrivePID();
            driveBySpeed = true;
        }
        drivePID.setReference(speed, CANSparkMax.ControlType.kVelocity);
    }

    /**
     * Set the direction and distance in encoder tics that the module should move. We use this for targeting when
     * the robot is stopped, and we are trying to get very fast rotation response and a very solid lock on the
     * target. This is far more reliable that trying to use a PID to control rotation speed to lock on a
     * target heading.
     *
     * @param targetDirection (AngleD) The direction from -pi to pi radians where 0.0 is towards the
     *                        front of the robot, and positive is clockwise.
     * @param deltaTics       (double) The number of tics the drive motor should move.
     */
    public void setDirectionAndDistance(AngleD targetDirection, double deltaTics) {
        setDirection(targetDirection);
        double targetTics = getDriveEncoderPosition() + deltaTics * speedMultiplier;

        if (driveBySpeed) {
            drivePID.setReference(0, CANSparkMax.ControlType.kVelocity);
            setDrivePosPID();
            driveBySpeed = false;
        }
        drivePID.setReference(targetTics, CANSparkMax.ControlType.kPosition);
    }
}
