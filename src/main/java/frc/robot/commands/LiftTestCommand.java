package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftSubsystem;


public class LiftTestCommand extends CommandBase {
    private final LiftSubsystem m_liftSubsystem = LiftSubsystem.getInstance();
    private final XboxController m_xbox;

    public LiftTestCommand(XboxController xbox) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_liftSubsystem);
        m_xbox = xbox;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_liftSubsystem.setLiftPositionsToTest();
//        m_liftSubsystem.setLeftPower(m_xbox.getLeftY());
//        m_liftSubsystem.setRightPower(m_xbox.getRightY());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
