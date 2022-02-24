package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CollectorSubsystem;


public class CollectorJerkCommand extends CommandBase {
    private final CollectorSubsystem m_collectorSubsystem = CollectorSubsystem.getInstance();
    private int m_cyclesElapsed = 0;
    private boolean m_backwards = false;
    private boolean m_done = true;

    public CollectorJerkCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_collectorSubsystem);
    }

    @Override
    public void initialize() {
        m_cyclesElapsed = 0;
        m_backwards = false;
        m_done = true;
    }

    @Override
    public void execute() {
        if (m_cyclesElapsed <= CollectorSubsystem.JERK_CYCLES && m_backwards) {
            m_collectorSubsystem.setPower(-m_collectorSubsystem.getCollectorPower());
        } else if (m_backwards) {
            m_backwards = false;
            m_cyclesElapsed = 0;
        }

        if (m_cyclesElapsed <= CollectorSubsystem.JERK_CYCLES && !m_backwards) {
            m_collectorSubsystem.setPower(m_collectorSubsystem.getCollectorPower());
        } else if (!m_backwards) {
            m_done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return m_done;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
