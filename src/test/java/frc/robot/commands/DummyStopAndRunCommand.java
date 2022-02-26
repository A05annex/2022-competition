package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class DummyStopAndRunCommand extends CommandBase {

    final private long startTime = System.currentTimeMillis();
    final private long endTime = startTime + 2000;

    public DummyStopAndRunCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
        System.out.printf("Instantiating command: class='%s'%n", this.getClass().getName());
        System.out.printf("          '%s':  ends at %d%n", this.getClass().getName(), endTime);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        System.out.print(".");
        System.out.flush();
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return (System.currentTimeMillis() > endTime);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.printf("%n          '%s':  ends after %.3f%n", this.getClass().getName(),
                (endTime-startTime)/1000.0);
    }
}
