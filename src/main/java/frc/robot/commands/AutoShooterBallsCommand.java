package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;


public class AutoShooterBallsCommand extends CommandBase {
    private final ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();

    public AutoShooterBallsCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_shooterSubsystem.setFrontShooter(ShooterSubsystem.AUTO_BALL_FRONT);
        m_shooterSubsystem.setRearShooter(ShooterSubsystem.AUTO_BALL_REAR);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
