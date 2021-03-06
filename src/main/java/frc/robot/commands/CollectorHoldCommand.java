package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.CollectorSubsystem;


public class CollectorHoldCommand extends CommandBase {
    private final CollectorSubsystem m_collectorSubsystem = CollectorSubsystem.getInstance();
    private final JoystickButton m_holdButton;
    private final JoystickButton m_reverseButton;

    /**
     * Default command for CollectorSubsystem, runs while a button is held.
     * @param holdButton (JoystickButton) Run at COLLECTOR_POWER while held.
     * @param reverseButton (JoystickButton) Run at -COLLECTOR_POWER while held.
     */
    public CollectorHoldCommand(JoystickButton holdButton, JoystickButton reverseButton) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_collectorSubsystem);
        m_holdButton = holdButton;
        m_reverseButton = reverseButton;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (m_reverseButton.get()) {
            m_collectorSubsystem.setPower(-CollectorSubsystem.COLLECTOR_POWER);
        } else if (m_holdButton.get()) {
            m_collectorSubsystem.setPower(CollectorSubsystem.COLLECTOR_POWER);
        } else {
            m_collectorSubsystem.setPower(0.0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
