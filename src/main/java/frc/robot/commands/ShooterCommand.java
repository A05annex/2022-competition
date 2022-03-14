package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class ShooterCommand extends CommandBase {
    private final ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();
    private final FeederSubsystem m_feederSubsystem = FeederSubsystem.getInstance();
    private int m_cyclesElapsed = 0;
    private final double m_frontPower;
    private final double m_rearPower;

    /**
     * Shoots one ball at the given speeds.
     * @param frontPower (double) Power to run the front shooter from -1.0 to 1.0.
     * @param rearPower (double) Power to run the rear shooter from -1.0 to 1.0.
     */
    public ShooterCommand(double frontPower, double rearPower) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_shooterSubsystem);
        addRequirements(m_feederSubsystem);
        m_frontPower = frontPower;
        m_rearPower = rearPower;
    }

    @Override
    public void initialize() {
        m_cyclesElapsed = 0;
    }

    @Override
    public void execute() {
        // set shooter speeds
        m_shooterSubsystem.setFrontShooter(m_frontPower);
        m_shooterSubsystem.setRearShooter(m_rearPower);

        // wait for shooter to rev up
        if (m_cyclesElapsed >= ShooterSubsystem.REV_CYCLES) {
            m_feederSubsystem.setPower(FeederSubsystem.FEEDER_POWER);
        }

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
