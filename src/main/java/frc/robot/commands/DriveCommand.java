package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import org.a05annex.frc.NavX;
import org.a05annex.frc.commands.A05DriveCommand;
import org.a05annex.frc.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;
import org.a05annex.util.Utl;


public class DriveCommand extends A05DriveCommand {
    private final LimelightSubsystem m_limelightSubsystem = LimelightSubsystem.getInstance();

    private final JoystickButton m_targetButton;

    /**
     * Default command for DriveSubsystem. Left stick moves the robot field-relatively, and right stick X rotates.
     * Contains driver constants for sensitivity, gain, and deadband.
     * @param xbox (XboxController) The drive xbox controller.
     * @param targetButton (JoystickButton) When held, rotates to the limelight target.
     */
    public DriveCommand(XboxController xbox, JoystickButton targetButton) {
        super(xbox);
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        m_targetButton = targetButton;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        conditionStick();

        // speed math
        double speed = computeSpeedFromStick();

        // rotate math
        double rotation;
        // are we targeting with the limelight?
        if (m_targetButton.get()) {
            // using the limelight targeting
            if (speed == 0) {
                // not moving, just setHeading and return
                m_driveSubsystem.setHeading(m_driveSubsystem.getFieldHeading().add(
                        m_limelightSubsystem.getTargetError()));
                return;
            } else {
                // moving, PID to target
                rotation = m_limelightSubsystem.getTargetError().getRadians() * Constants.TARGET_kP;
            }
        } else {
            rotation = computeRotationFromStick(speed);
        }

        // find direction, if speed is close to 0 rotation will be zeroed
        AngleD direction = new AngleD(AngleUnit.RADIANS, Math.atan2(m_stickX, m_stickY));

        m_driveSubsystem.swerveDriveFieldRelative(direction, speed, rotation);
    }
}
