package frc.robot.subsystems;

import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;

/**
 * Our interface specific to controlling the swerve drive. This is abstracted into an interface, so we can build
 * mock implementations of the drive subsystem for testing and so that we can build swerve drive commands
 */
public interface ISwerveDrive {

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
    public void setFieldPosition(double fieldX, double fieldY, AngleD heading);

    /**
     * Run the swerve drive with the specified {@code  forward}, {@code strafe}, and {@code rotation} chassis
     * relative components.
     *
     * @param forward  Drive forward. From -1 (full backwards) to 1 (full forwards.
     * @param strafe   Strafe right. From -1 (full left)  to 1 (full right).
     * @param rotation Clockwise rotation. From -1 (full counter-clockwise) to 1 (full clockwise).
     */
    void swerveDriveComponents(double forward, double strafe, double rotation);

    /**
     * Prepare the swerve drive to run with the swerve drive with the specified {@code  forward}, {@code strafe},
     * and {@code rotation} chassis relative components. 'Prepare', in this context, means orient all the modules,
     * so they are ready to perform this command but set the module speeds to 0.0; In this way movement can start
     * smoothly without additional module reorientation. This method is used to initialize the robot before the
     * start of an autonomous path.
     *
     * @param forward  Drive forward. From -1 (full backwards) to 1 (full forwards.
     * @param strafe   Strafe right. From -1 (full left)  to 1 (full right).
     * @param rotation Clockwise rotation. From -1 (full counter-clockwise) to 1 (full clockwise).
     */
    void prepareForDriveComponents(double forward, double strafe, double rotation);

    /**
     * Swerve drive with a robot-relative direction, a speed and a rotation speed.
     *
     * @param chassisDirection (AngleConstantD) The robot chassis relative direction in radians from -PI to
     *                         PI where 0.0 is towards the front of the robot, and positive is clockwise.
     * @param speed            (double) Speed from 0.0 to 1.0.
     * @param rotation         (double) Clockwise rotation speed from -1.0 to 1.0.
     */
    void swerveDrive(AngleConstantD chassisDirection, double speed, double rotation);

    /**
     * Swerve drive with a field-relative direction, a speed and a rotation.
     *
     * @param fieldDirection (AngleD) The direction in radians from -PI to PI where 0.0 is away from the
     *                       driver, and positive is clockwise.
     * @param speed          (double) Speed from 0.0 to 1.0.
     * @param rotation       (double) Clockwise rotation speed from -1.0 to 1.0.
     */
    void swerveDriveFieldRelative(AngleConstantD fieldDirection, double speed, double rotation);

    /**
     * Rotate the chassis to the specified heading with no field translation. This controls the module using distance
     * (i.e. moving a specified number of ticks) rather than speed because this adjustment of heading is faster
     * and more reliable.
     *
     * @param targetHeading (AngleConstantD) The desired chassis heading on the field.
     */
    void setHeading(AngleConstantD targetHeading);
}
