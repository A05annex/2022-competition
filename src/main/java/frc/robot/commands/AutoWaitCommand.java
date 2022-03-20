package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;


public class AutoWaitCommand extends CommandBase {
    private int m_cyclesElapsed = 0;

    public AutoWaitCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
    }

    @Override
    public void initialize() {
        m_cyclesElapsed = 0;
    }

    @Override
    public void execute() {
        m_cyclesElapsed++;
    }

    @Override
    public boolean isFinished() {
        return m_cyclesElapsed >= Constants.AUTO_WAIT_CYCLES;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
