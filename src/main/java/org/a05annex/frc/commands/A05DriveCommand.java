package org.a05annex.frc.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.NavX;
import org.a05annex.frc.subsystems.DriveSubsystem;
import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;
import org.a05annex.util.Utl;

public class A05DriveCommand extends CommandBase {
    protected final DriveSubsystem m_driveSubsystem = DriveSubsystem.getInstance();

    protected final NavX m_navx = NavX.getInstance();

    protected final XboxController m_xbox;

    protected double m_gain;

    protected double m_stickY;
    protected double m_stickX;
    protected double m_stickRotate;

    // save last stick values to limit rate of change
    protected double m_lastStickX = 0.0;
    protected double m_lastStickY = 0.0;
    protected double m_lastStickRotate = 0.0;

    // deadband of drive and rotate joysticks
    public static double DRIVE_DEADBAND = 0.05;
    public static double ROTATE_DEADBAND = 0.05;

    // sensitivity and gain
    public static double DRIVE_SPEED_SENSITIVITY = 2.0;
    public static double DRIVE_SPEED_GAIN = 0.7;
    public static double ROTATE_SENSITIVITY = 1.5;
    public static double ROTATE_GAIN = 0.5;

    // boost button gain, slow, and trigger threshold
    public static final double DRIVE_BOOST_GAIN = 1.0;
    public static final double DRIVE_SLOW_GAIN = 0.3;
    public static final double BOOST_TRIGGER_THRESHOLD = 0.5;

    // maximum change in joystick value per 20ms for speed and rotation
    public static double DRIVE_MAX_SPEED_INC = 0.075;
    public static double DRIVE_MAX_ROTATE_INC = 0.075;

    public A05DriveCommand(XboxController xbox) {
        addRequirements(m_driveSubsystem);
        m_xbox = xbox;
    }

    @Override
    public void execute() {
        conditionStick();

        // speed math
        double speed = computeSpeedFromStick();

        // rotate math
        double rotation = computeRotationFromStick(speed);

        // find direction, if speed is close to 0 rotation will be zeroed
        AngleD direction = new AngleD(AngleUnit.RADIANS, Math.atan2(m_stickX, m_stickY));

        m_driveSubsystem.swerveDriveFieldRelative(direction, speed, rotation);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }

    protected void conditionStick() {
        // if pressing boost button, set gain to boost gain
        m_gain = DRIVE_SPEED_GAIN;
        if ((A05Constants.getIncreaseGainAxis() != null) &&
                (m_xbox.getRawAxis(A05Constants.getIncreaseGainAxis().value)) >= BOOST_TRIGGER_THRESHOLD) {
            m_gain = DRIVE_BOOST_GAIN;
        } else if ((A05Constants.getDecreaseGainAxis() != null) &&
                (m_xbox.getRawAxis(A05Constants.getDecreaseGainAxis().value)) >= BOOST_TRIGGER_THRESHOLD) {
            m_gain = DRIVE_SLOW_GAIN;
        }

        // get stick values
        // left stick Y for forward/backward speed
        m_stickY = -m_xbox.getLeftY(); // Y is inverted so up is 1 and down is -1

        // left stick X for strafe speed
        m_stickX = m_xbox.getLeftX();

        // right stick X for rotation speed
        m_stickRotate = m_xbox.getRightX();

        // limit rate of change of stick values to reduce skidding
        m_stickX = Utl.clip(m_stickX, m_lastStickX - DRIVE_MAX_SPEED_INC,
                m_lastStickX + DRIVE_MAX_SPEED_INC);
        m_stickY = Utl.clip(m_stickY, m_lastStickY - DRIVE_MAX_SPEED_INC,
                m_lastStickY + DRIVE_MAX_SPEED_INC);
        m_stickRotate = Utl.clip(m_stickRotate, m_lastStickRotate - DRIVE_MAX_ROTATE_INC,
                m_lastStickRotate + DRIVE_MAX_ROTATE_INC);

        // set last stick values
        m_lastStickX = m_stickX;
        m_lastStickY = m_stickY;
        m_lastStickRotate = m_stickRotate;
    }

    protected double computeSpeedFromStick() {
        // speed math
        double speed;
        double distance = Utl.length(m_stickY, m_stickX);
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
        return Math.pow(speed, DRIVE_SPEED_SENSITIVITY) * m_gain;
    }

    protected double computeRotationFromStick(double speed) {
        double rotation;
        // take out rotation sign and store it for later
        double rotationMult = (m_stickRotate < 0.0) ? -1.0 : 1.0;
        m_stickRotate = Math.abs(m_stickRotate);
        // are we rotating?
        if (m_stickRotate < ROTATE_DEADBAND) {
            // no rotate, keep current heading or 0 if no NavX
            NavX.HeadingInfo headingInfo = m_navx.getHeadingInfo();
            if (headingInfo != null) {
                rotation = new AngleD(headingInfo.expectedHeading).subtract(new AngleD(headingInfo.heading))
                        .getRadians() * Constants.DRIVE_ORIENTATION_kP;
                // clip and add speed multiplier
                return Utl.clip(rotation, -0.5, 0.5) * speed;
            } else {
                // no NavX
                return 0.0;
            }
        } else {
            // rotating
            // adjust for deadband
            rotation = (m_stickRotate - ROTATE_DEADBAND) / (1.0 - ROTATE_DEADBAND);
            // update expected heading
            m_navx.setExpectedHeadingToCurrent();
            // add sensitivity, gain and sign
            return Math.pow(rotation, ROTATE_SENSITIVITY) * ROTATE_GAIN * rotationMult;
        }
    }
}
