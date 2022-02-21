package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CollectorSubsystem;


public class CollectorCommand extends CommandBase {

    private final CollectorSubsystem m_collectorSubsystem = CollectorSubsystem.getInstance();

    /**
     * Constantly runs the collector at the given power.
     */
    public CollectorCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_collectorSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_collectorSubsystem.setPower(m_collectorSubsystem.collectorPowerTest);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_collectorSubsystem.setPower(0.0);
    }
}
