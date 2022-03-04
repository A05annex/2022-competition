package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CollectorSubsystem;


public class AutoCollectorOnCommand extends CommandBase {
    private final CollectorSubsystem m_collectorSubsystem = CollectorSubsystem.getInstance();

    public AutoCollectorOnCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
//        addRequirements(m_collectorSubsystem); path planner won't work if this is here
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_collectorSubsystem.setPower(CollectorSubsystem.COLLECTOR_POWER);
    }

    @Override
    public boolean isFinished() {
        return !RobotState.isAutonomous();
    }

    @Override
    public void end(boolean interrupted) {
        m_collectorSubsystem.setPower(0.0);
    }
}
