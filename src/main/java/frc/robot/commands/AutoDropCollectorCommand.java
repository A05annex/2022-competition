package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftSubsystem;


public class AutoDropCollectorCommand extends CommandBase {
    private final LiftSubsystem m_liftSubsystem = LiftSubsystem.getInstance();
    private int m_cyclesElapsed = 0;
    private boolean m_done = false;

    /**
     * Lifts the first hanger bar to drop the collector in auto. Used as a stop-and-run in the path planner.
     */
    public AutoDropCollectorCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_liftSubsystem);
    }

    @Override
    public void initialize() {
        m_cyclesElapsed = 0;
        m_done = false;
    }

    @Override
    public void execute() {
        if (m_cyclesElapsed <= LiftSubsystem.LIFT_AUTO_CYCLES) {
            m_liftSubsystem.setLeftPower(LiftSubsystem.LIFT_POWER);
        } else {
            m_done = true;
        }

        m_cyclesElapsed++;
    }

    @Override
    public boolean isFinished() {
        return m_done;
    }

    @Override
    public void end(boolean interrupted) {
        m_liftSubsystem.setLeftPower(0.0);
    }
}
