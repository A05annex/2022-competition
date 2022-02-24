package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.NavX;
import frc.robot.subsystems.DriveSubsystem;
import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;
import org.a05annex.util.Utl;


public class DriveCommand extends CommandBase {
    private final DriveSubsystem m_driveSubsystem = DriveSubsystem.getInstance();

    private final XboxController m_xbox;
    private final NavX m_navx = NavX.getInstance();

    // save last stick values to limit rate of change
    private double m_lastStickX = 0.0;
    private double m_lastStickY = 0.0;
    private double m_lastStickRotate = 0.0;

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

    public DriveCommand(XboxController xbox) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.m_driveSubsystem);
        m_xbox = xbox;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // get stick values
        // left stick Y for forward/backward speed
        double stickY = -m_xbox.getLeftY(); // Y is inverted so up is 1 and down is -1

        // left stick X for strafe speed
        double stickX = m_xbox.getLeftX();

        // right stick X for rotation speed
        double stickRotate = m_xbox.getRightX();

        // limit rate of change of stick values to reduce skidding
        stickX = Utl.clip(stickX, m_lastStickX - DRIVE_MAX_SPEED_INC,
                m_lastStickX + DRIVE_MAX_SPEED_INC);
        stickY = Utl.clip(stickY, m_lastStickY - DRIVE_MAX_SPEED_INC,
                m_lastStickY + DRIVE_MAX_SPEED_INC);
        stickRotate = Utl.clip(stickRotate, m_lastStickRotate - DRIVE_MAX_ROTATE_INC,
                m_lastStickRotate + DRIVE_MAX_ROTATE_INC);

        // set last stick values
        m_lastStickX = stickX;
        m_lastStickY = stickY;
        m_lastStickRotate = stickRotate;

        // speed math
        double speed;
        double distance = Utl.length(stickY,stickX);
        // deadband
        if (distance < DRIVE_DEADBAND) {
            speed = 0.0;
        } else {
            if (distance > 1.0) {
                distance = 1.0;
            }
            speed = (distance - DRIVE_DEADBAND) / (1.0 - DRIVE_DEADBAND);
        }
        // add gain and sensitivity
        speed = Math.pow(speed, DRIVE_SPEED_SENSITIVITY) * DRIVE_SPEED_GAIN;

        // rotate math
        double rotation;
        // take out rotation sign and store it for later
        double rotationMult = (stickRotate < 0.0) ? -1.0 : 1.0;
        stickRotate = Math.abs(stickRotate);
        // are we rotating?
        if (stickRotate < ROTATE_DEADBAND) {
            // no rotate, keep current heading or 0 if no NavX
            NavX.HeadingInfo headingInfo = m_navx.getHeadingInfo();
            if (headingInfo != null) {
                rotation = new AngleD(headingInfo.expectedHeading).subtract(new AngleD(headingInfo.heading))
                        .getRadians() * Constants.DRIVE_ORIENTATION_kP;
                // clip and add speed multiplier
                rotation = Utl.clip(rotation, -0.5, 0.5) * speed;
            } else {
                // no NavX
                rotation = 0.0;
            }
        } else {
            // rotating
            // adjust for deadband
            rotation = (stickRotate - ROTATE_DEADBAND) / (1.0 - ROTATE_DEADBAND);
            // update expected heading
            m_navx.setExpectedHeadingToCurrent();
            // add sensitivity, gain and sign
            rotation = Math.pow(rotation, ROTATE_SENSITIVITY) * ROTATE_GAIN * rotationMult;
        }

        // find direction, if speed is close to 0 rotation will be zeroed
        AngleD direction = new AngleD(AngleUnit.RADIANS, Math.atan2(stickX, stickY));

        m_driveSubsystem.swerveDriveFieldRelative(direction, speed, rotation);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
