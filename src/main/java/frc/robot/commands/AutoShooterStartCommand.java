package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class AutoShooterStartCommand extends CommandBase {
    private final ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();
    private final FeederSubsystem m_feederSubsystem = FeederSubsystem.getInstance();
    private int m_cyclesElapsed = 0;
    private boolean m_done = false;
    private final int REV_CYCLES = 25; // 0.5 seconds
    private final int WAIT_CYCLES = 100; // 2 seconds total

    public AutoShooterStartCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_shooterSubsystem);
        addRequirements(m_feederSubsystem);
    }

    @Override
    public void initialize() {
        m_cyclesElapsed = 0;
        m_done = false;
    }

    @Override
    public void execute() {
        // set shooter speeds
        m_shooterSubsystem.setFrontShooter(ShooterSubsystem.AUTO_START_FRONT);
        m_shooterSubsystem.setRearShooter(ShooterSubsystem.AUTO_START_REAR);

        // wait for shooter to rev up
        if (m_cyclesElapsed >= REV_CYCLES) {
            m_feederSubsystem.setPower(FeederSubsystem.FEEDER_POWER);
        }

        m_cyclesElapsed++;
    }

    @Override
    public boolean isFinished() {
        return m_cyclesElapsed >= WAIT_CYCLES;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.setFrontShooter(0.0);
        m_shooterSubsystem.setRearShooter(0.0);
        m_feederSubsystem.setPower(0.0);
    }
}
