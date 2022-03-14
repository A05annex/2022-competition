package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CollectorSubsystem;


public class CollectorJerkCommand extends CommandBase {
    private final CollectorSubsystem m_collectorSubsystem = CollectorSubsystem.getInstance();
    private int m_cyclesElapsed = 0;
    private int m_state = 0;

    /**
     * Jerks the collector forward and backward, then ends. Uses constants in CollectorSubsystem.
     */
    public CollectorJerkCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_collectorSubsystem);
    }

    @Override
    public void initialize() {
        m_cyclesElapsed = 0;
        // if stopped, skip first state
        if (m_collectorSubsystem.getPower() == 0.0) {
            m_state = 1;
        } else {
            m_state = 0;
        }
    }

    @Override
    public void execute() {
        // stop to let ball rest if running
        if (m_cyclesElapsed <= CollectorSubsystem.STOP_CYCLES && m_state == 0) {
            m_collectorSubsystem.setPower(0.0);
        } else if (m_state == 0) {
            m_state = 1;
            m_cyclesElapsed = 0;
        }

        // run backwards
        if (m_cyclesElapsed <= CollectorSubsystem.BACK_CYCLES && m_state == 1) {
            m_collectorSubsystem.setPower(-CollectorSubsystem.BACK_POWER);
        } else if (m_state == 1) {
            m_state = 2;
            m_cyclesElapsed = 0;
        }

        // run forwards
        if (m_cyclesElapsed <= CollectorSubsystem.FORWARD_CYCLES && m_state == 2) {
            m_collectorSubsystem.setPower(CollectorSubsystem.FORWARD_POWER);
        } else if (m_state == 2) {
            m_state = 3;
            m_cyclesElapsed = 0;
        }

        m_cyclesElapsed++;
    }

    @Override
    public boolean isFinished() {
        return m_state == 3;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
