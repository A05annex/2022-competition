package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;


public class FeederCommand extends CommandBase {
    private final FeederSubsystem m_feederSubsystem = FeederSubsystem.getInstance();
    private final double m_power;

    public FeederCommand(double power) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_feederSubsystem);
        m_power = power;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_feederSubsystem.setPower(m_power);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_feederSubsystem.setPower(0.0);
    }
}
