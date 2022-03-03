package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;


public class AutoShooterBallsCommand extends CommandBase {
    private final ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();
    private final FeederSubsystem m_feederSubsystem = FeederSubsystem.getInstance();
    private final CollectorSubsystem m_collectorSubsystem = CollectorSubsystem.getInstance();
    private final DriveSubsystem m_driveSubsystem = DriveSubsystem.getInstance();
    private final LimelightSubsystem m_limelightSubsystem = LimelightSubsystem.getInstance();
    private int m_cyclesElapsed = 0;

    public AutoShooterBallsCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_shooterSubsystem);
        addRequirements(m_feederSubsystem);
        addRequirements(m_collectorSubsystem);
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void initialize() {
        m_cyclesElapsed = 0;
        m_collectorSubsystem.setPower(0.0);
    }

    @Override
    public void execute() {
        // set shooter speeds
        m_shooterSubsystem.setFrontShooter(ShooterSubsystem.AUTO_BALL_FRONT);
        m_shooterSubsystem.setRearShooter(ShooterSubsystem.AUTO_BALL_REAR);

        // wait for shooter to rev up
        if (m_cyclesElapsed >= ShooterSubsystem.REV_CYCLES) {
            m_feederSubsystem.setPower(FeederSubsystem.FEEDER_POWER);
        }

        // limelight target
        m_driveSubsystem.setHeading(m_driveSubsystem.getFieldHeading().add(
                m_limelightSubsystem.getTargetError()));

        m_cyclesElapsed++;
    }

    @Override
    public boolean isFinished() {
        return m_cyclesElapsed >= ShooterSubsystem.WAIT_CYCLES;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.setFrontShooter(0.0);
        m_shooterSubsystem.setRearShooter(0.0);
        m_feederSubsystem.setPower(0.0);
    }
}
