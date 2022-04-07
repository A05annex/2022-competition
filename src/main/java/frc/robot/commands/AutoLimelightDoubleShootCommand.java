package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightCalibrationPoint;
import frc.robot.subsystems.*;


public class AutoLimelightDoubleShootCommand extends CommandBase {
    private final CollectorSubsystem m_collectorSubsystem = CollectorSubsystem.getInstance();
    private final ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();
    private final FeederSubsystem m_feederSubsystem = FeederSubsystem.getInstance();
    private final LimelightSubsystem m_limelightSubsystem = LimelightSubsystem.getInstance();
    private final DriveSubsystem m_driveSubsystem = DriveSubsystem.getInstance();

    private LimelightCalibrationPoint m_shooterSpeeds;
    private int m_totalCyclesElapsed = 0;
    private int m_stateCyclesElapsed = 0;
    private STATE m_state = STATE.SPINUP;
    private boolean m_done = false;

    /**
     * Shoots two balls at speeds given by the Limelight. If no target or outside shootable range, does not run.
     */
    public AutoLimelightDoubleShootCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_collectorSubsystem, m_shooterSubsystem, m_driveSubsystem);
    }

    @Override
    public void initialize() {
        m_totalCyclesElapsed = 0;
        m_stateCyclesElapsed = 0;
        m_state = STATE.SPINUP;
        m_done = false;
        // give me control of collector
        m_shooterSubsystem.setIsShooting(true);
    }

    private enum STATE {
        SPINUP,
        SHOOT_1,
        JERK_REVERSE,
        JERK_FORWARD,
        SHOOT_2
    }

    @Override
    public void execute() {
        // keep total time, end if over
        m_totalCyclesElapsed++;
        if (m_totalCyclesElapsed >= ShooterSubsystem.DOUBLE_CYCLES) {
            m_done = true;
        }

        if (!m_done) {
            // state machine (i think?)
            if (m_state == STATE.SPINUP && m_stateCyclesElapsed > ShooterSubsystem.AUTO_REV_CYCLES) {
                m_state = STATE.SHOOT_1;
                m_stateCyclesElapsed = 0;
            } else if (m_state == STATE.SHOOT_1 && m_stateCyclesElapsed > ShooterSubsystem.SHOOT_CYCLES) {
                m_state = STATE.JERK_REVERSE;
                m_stateCyclesElapsed = 0;
            } else if (m_state == STATE.JERK_REVERSE && m_stateCyclesElapsed > CollectorSubsystem.BACK_CYCLES) {
                m_state = STATE.JERK_FORWARD;
                m_stateCyclesElapsed = 0;
            } else if (m_state == STATE.JERK_FORWARD && m_stateCyclesElapsed > CollectorSubsystem.FORWARD_CYCLES) {
                m_state = STATE.SHOOT_2;
                m_stateCyclesElapsed = 0;
            } else if (m_state == STATE.SHOOT_2 && m_stateCyclesElapsed > ShooterSubsystem.SETTLE_CYCLES) {
                m_state = STATE.JERK_REVERSE; // go back to jerking
                m_stateCyclesElapsed = 0;
            }
            m_stateCyclesElapsed++;

            // recalculate shooter speeds
            m_shooterSpeeds = m_limelightSubsystem.getShooterSpeeds();
            // run shooters no matter state
            if (m_limelightSubsystem.canShoot() == LimelightSubsystem.CAN_SHOOT.YES) {
                m_shooterSubsystem.setFrontShooter(m_shooterSpeeds.frontSpeed);
                m_shooterSubsystem.setRearShooter(m_shooterSpeeds.rearSpeed);
            } else {
                m_shooterSubsystem.setFrontShooter(ShooterSubsystem.AUTO_BALL_FRONT);
                m_shooterSubsystem.setRearShooter(ShooterSubsystem.AUTO_BALL_REAR);
            }

            // limelight target
            m_driveSubsystem.setHeading(m_driveSubsystem.getFieldHeading().add(
                    m_limelightSubsystem.getTargetError()));

            switch (m_state) {
                case SPINUP:
                    // keep collecting
                    m_collectorSubsystem.setPower(CollectorSubsystem.COLLECTOR_POWER);
                    break;
                case SHOOT_1:
                    // stop collector and feed
                    m_collectorSubsystem.setPower(0.0);
                    m_feederSubsystem.setPower(FeederSubsystem.FEEDER_POWER);
                    break;
                case JERK_REVERSE:
                    // feed and jerk backward
                    m_collectorSubsystem.setPower(-CollectorSubsystem.BACK_POWER + CollectorSubsystem.DOUBLE_BUMP);
                    m_feederSubsystem.setPower(FeederSubsystem.FEEDER_POWER);
                    break;
                case JERK_FORWARD:
                    // feed and jerk forward
                    m_collectorSubsystem.setPower(CollectorSubsystem.FORWARD_POWER - CollectorSubsystem.DOUBLE_BUMP);
                    m_feederSubsystem.setPower(FeederSubsystem.FEEDER_POWER);
                    break;
                case SHOOT_2:
                    // stop collector and feed
                    m_collectorSubsystem.setPower(0.0);
                    m_feederSubsystem.setPower(FeederSubsystem.FEEDER_POWER);
                    break;
            }
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
        m_shooterSubsystem.setIsShooting(false);
    }
}
