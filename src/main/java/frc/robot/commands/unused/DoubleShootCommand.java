package frc.robot.commands.unused;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class DoubleShootCommand extends CommandBase {
    private final CollectorSubsystem m_collectorSubsystem = CollectorSubsystem.getInstance();
    private final ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();
    private final FeederSubsystem m_feederSubsystem = FeederSubsystem.getInstance();
    private int m_feederCyclesElapsed = 0;
    private int m_jerkCyclesElapsed = 0;
    private int m_state = 0;
    private boolean m_done = false;
    private final double m_frontPower;
    private final double m_rearPower;

    /**
     * Shoots once, jerks the collector, then shoots again.
     * @param frontPower (double) Power to run the front shooter from -1.0 to 1.0.
     * @param rearPower (double) Power to run the rear shooter from -1.0 to 1.0.
     */
    public DoubleShootCommand(double frontPower, double rearPower) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_collectorSubsystem, m_shooterSubsystem);
        m_frontPower = frontPower;
        m_rearPower = rearPower;
    }

    @Override
    public void initialize() {
        m_feederCyclesElapsed = 0;
        m_jerkCyclesElapsed = 0;
        m_state = 0;
        m_done = false;
    }

    @Override
    public void execute() {
        // start shooters
        m_shooterSubsystem.setFrontShooter(m_frontPower);
        m_shooterSubsystem.setRearShooter(m_rearPower);

        // start feeder after REV_CYCLES
        if (m_feederCyclesElapsed >= ShooterSubsystem.REV_CYCLES) {
            m_feederSubsystem.setPower(FeederSubsystem.FEEDER_POWER);
        }

        // after WAIT_CYCLES, start collector jerk
        if (m_feederCyclesElapsed >= ShooterSubsystem.WAIT_CYCLES) {
            m_jerkCyclesElapsed++;
        }

        // run backwards
        if (m_jerkCyclesElapsed != 0 && m_jerkCyclesElapsed <= CollectorSubsystem.BACK_CYCLES && m_state == 0) {
            m_collectorSubsystem.setPower(-CollectorSubsystem.BACK_POWER);
        } else if (m_jerkCyclesElapsed != 0 && m_state == 0) {
            m_state = 1;
            m_jerkCyclesElapsed = 0;
        }

        // run forwards
        if (m_jerkCyclesElapsed <= CollectorSubsystem.FORWARD_CYCLES && m_state == 1) {
            m_collectorSubsystem.setPower(CollectorSubsystem.FORWARD_POWER);
        } else if (m_state == 1) {
            m_state = 2;
            m_jerkCyclesElapsed = 0;
        }

        // wait for ball to settle and shoot
        if (m_jerkCyclesElapsed >= ShooterSubsystem.SETTLE_CYCLES && m_state == 2) {
            m_done = true;
        }

        m_feederCyclesElapsed++;
    }

    @Override
    public boolean isFinished() {
        return m_done;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.setFrontShooter(0.0);
        m_shooterSubsystem.setRearShooter(0.0);
        m_feederSubsystem.setPower(0.0);
        m_collectorSubsystem.setPower(0.0);
    }
}
