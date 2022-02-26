package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class DummyStopAndRunCommand extends CommandBase {

    final private long endTime = System.currentTimeMillis() + 2000;

    public DummyStopAndRunCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
        System.out.printf("Instantiating command: class='%s'%n", this.getClass().getName());
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return (System.currentTimeMillis() > endTime);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
