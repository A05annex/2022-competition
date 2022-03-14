package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.LimelightCalibrationPoint;


public class LimelightDoubleShootCommand extends CommandBase {
    private final CollectorSubsystem m_collectorSubsystem = CollectorSubsystem.getInstance();
    private final ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();
    private final FeederSubsystem m_feederSubsystem = FeederSubsystem.getInstance();
    private final LimelightSubsystem m_limelightSubsystem = LimelightSubsystem.getInstance();

    private LimelightCalibrationPoint m_shooterSpeeds;
    private int m_feederCyclesElapsed = 0;
    private int m_jerkCyclesElapsed = 0;
    private int m_state = 0;
    private boolean m_done = false;

    /**
     * Shoots two balls at speeds given by the Limelight. If no target or outside shootable range, does not run.
     */
    public LimelightDoubleShootCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_collectorSubsystem, m_shooterSubsystem);
    }

    @Override
    public void initialize() {
        m_feederCyclesElapsed = 0;
        m_jerkCyclesElapsed = 0;
        m_state = 0;
        m_done = false;
        m_shooterSpeeds = m_limelightSubsystem.getShooterSpeeds();
        if (m_shooterSpeeds == null) {
            m_done = true;
        }
    }

    @Override
    public void execute() {
        if (!m_done) {
            // start shooters
            m_shooterSubsystem.setFrontShooter(m_shooterSpeeds.frontSpeed);
            m_shooterSubsystem.setRearShooter(m_shooterSpeeds.rearSpeed);

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
