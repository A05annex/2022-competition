package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CollectorSubsystem;


public class CollectorCommand extends CommandBase {

    private final CollectorSubsystem m_collectorSubsystem = CollectorSubsystem.getInstance();
    private final double m_power;

    /**
     * Constantly runs the collector at the given power.
     * @param power (double) Power to set the collector to. Between -1.0 and 1.0.
     */
    public CollectorCommand(double power) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_collectorSubsystem);
        m_power = power;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_collectorSubsystem.setCollectorPower(m_power);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_collectorSubsystem.setCollectorPower(0.0);
    }
}
