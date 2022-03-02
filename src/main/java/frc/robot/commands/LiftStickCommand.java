package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftSubsystem;


public class LiftStickCommand extends CommandBase {
    private final LiftSubsystem m_liftSubsystem = LiftSubsystem.getInstance();
    private final XboxController m_xbox;
    private final double HANG_DEADBAND = 0.05;

    public LiftStickCommand(XboxController xbox) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_liftSubsystem);
        m_xbox = xbox;
    }

    @Override
    public void initialize() {
        m_liftSubsystem.zeroEncoders();
    }

    @Override
    public void execute() {
        double xboxRight = -m_xbox.getRightY();
        double xboxLeft = -m_xbox.getLeftY();

        if (xboxRight > -HANG_DEADBAND && xboxRight < HANG_DEADBAND) {
            m_liftSubsystem.setRightPosition(m_liftSubsystem.getRightPosition());
        } else {
            m_liftSubsystem.setRightPower(xboxRight);
        }

        if (xboxLeft > -HANG_DEADBAND && xboxLeft < HANG_DEADBAND) {
            m_liftSubsystem.setLeftPosition(m_liftSubsystem.getLeftPosition());
        } else {
            m_liftSubsystem.setLeftPower(xboxLeft);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_liftSubsystem.setLeftPosition(m_liftSubsystem.getLeftPosition());
        m_liftSubsystem.setRightPosition(m_liftSubsystem.getRightPosition());
    }
}
