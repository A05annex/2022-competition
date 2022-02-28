package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.FeederSubsystem;


public class FeederCommand extends CommandBase {
    private final FeederSubsystem m_feederSubsystem = FeederSubsystem.getInstance();
    private final JoystickButton m_forwardButton;
    private final JoystickButton m_reverseButton;

    /**
     * Runs the feeder at the constant speed in FeederSubsystem.
     *
     * @param forwardButton Runs forward when pressed.
     * @param reverseButton Reverses when pressed.
     */
    public FeederCommand(JoystickButton forwardButton, JoystickButton reverseButton) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_feederSubsystem);
        m_forwardButton = forwardButton;
        m_reverseButton = reverseButton;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (m_forwardButton.get()) {
            m_feederSubsystem.setPower(m_feederSubsystem.getFeederPower());
        } else if (m_reverseButton.get()) {
            m_feederSubsystem.setPower(-m_feederSubsystem.getFeederPower());
        } else {
            m_feederSubsystem.setPower(0.0);
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_feederSubsystem.setPower(0.0);
    }
}
