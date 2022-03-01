package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.CollectorSubsystem;


public class CollectorToggleCommand extends CommandBase {

    private final CollectorSubsystem m_collectorSubsystem = CollectorSubsystem.getInstance();
    private final JoystickButton m_toggleButton;

    private boolean m_isRunning = false;
    private boolean m_lastToggleButton = false;

    /**
     * Runs the collector at the constant speed in CollectorSubsystem.
     *
     * @param toggleButton Toggles the collector on/off.
     */
    public CollectorToggleCommand(JoystickButton toggleButton) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_collectorSubsystem);
        m_toggleButton = toggleButton;
    }

    @Override
    public void initialize() {
        //m_isRunning = false; keep last state
        m_lastToggleButton = false;
    }

    @Override
    public void execute() {
        if (m_toggleButton.get() && !m_lastToggleButton) {
            m_isRunning = !m_isRunning;
        }

        if (m_isRunning) {
            m_collectorSubsystem.setPower(CollectorSubsystem.COLLECTOR_POWER);
        } else {
            m_collectorSubsystem.setPower(0.0);
        }

        m_lastToggleButton = m_toggleButton.get();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        //m_collectorSubsystem.setPower(0.0);
    }
}
