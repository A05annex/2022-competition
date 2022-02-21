package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;


public class ShooterCommand extends CommandBase {

    private final ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();

//    private double m_frontSpeed;
//    private double m_rearSpeed;

    public ShooterCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_shooterSubsystem);
//        m_frontSpeed = frontSpeed;
//        m_rearSpeed = rearSpeed;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_shooterSubsystem.setFrontShooter(m_shooterSubsystem.frontShooterTestSpeed);
        m_shooterSubsystem.setRearShooter(m_shooterSubsystem.rearShooterTestSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.setFrontShooter(0.0);
        m_shooterSubsystem.setRearShooter(0.0);
    }
}
