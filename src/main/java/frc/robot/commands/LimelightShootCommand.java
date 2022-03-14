package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.LimelightCalibrationPoint;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class LimelightShootCommand extends CommandBase {
    private final ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();
    private final FeederSubsystem m_feederSubsystem = FeederSubsystem.getInstance();
    private final LimelightSubsystem m_limelightSubsystem = LimelightSubsystem.getInstance();
    private int m_cyclesElapsed = 0;
    private LimelightCalibrationPoint m_shooterSpeeds = null;
    private boolean m_done = false;

    /**
     * Shoots one ball at speed given by the Limelight. If no target or outside shootable range, does not run.
     */
    public LimelightShootCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_shooterSubsystem);
        addRequirements(m_feederSubsystem);
    }

    @Override
    public void initialize() {
        m_cyclesElapsed = 0;
        m_done = false;
        m_shooterSpeeds = m_limelightSubsystem.getShooterSpeeds();
        if (m_shooterSpeeds == null) {
            m_done = true;
        }
    }

    @Override
    public void execute() {
        if (!m_done) {
            // set shooter speeds
            m_shooterSubsystem.setFrontShooter(m_shooterSpeeds.frontSpeed);
            m_shooterSubsystem.setRearShooter(m_shooterSpeeds.rearSpeed);

            // wait for shooter to rev up
            if (m_cyclesElapsed >= ShooterSubsystem.REV_CYCLES) {
                m_feederSubsystem.setPower(FeederSubsystem.FEEDER_POWER);
            }

            m_cyclesElapsed++;
        }
    }

    @Override
    public boolean isFinished() {
        return (m_cyclesElapsed >= ShooterSubsystem.WAIT_CYCLES) || m_done;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.setFrontShooter(0.0);
        m_shooterSubsystem.setRearShooter(0.0);
        m_feederSubsystem.setPower(0.0);
    }
}
