package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftSubsystem;


public class LiftTestCommand extends CommandBase {
    private final LiftSubsystem m_liftSubsystem = LiftSubsystem.getInstance();

    public LiftTestCommand(double position) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_liftSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_liftSubsystem.setLiftPositionsToTest();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
