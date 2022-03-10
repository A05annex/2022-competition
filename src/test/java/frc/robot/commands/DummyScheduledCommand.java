package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * This is a simple dummy command for testing scheduled commands in the path. After it is scheduled
 * it should be initialized, execute for one command cycle, and then end. Note that the
 * {@link Command#runsWhenDisabled()} is overridden to return {@link true} so the command will
 * schedule and run in the test environment in which the scheduler thinks the robot is diabled.
 */
public class DummyScheduledCommand extends CommandBase {
    static int instantiateCt = 0;
    static int initializeCt = 0;
    static int executeCt = 0;
    static int endCt = 0;

    public static void resetCounts() {
        instantiateCt = 0;
        initializeCt = 0;
        executeCt = 0;
        endCt = 0;
    }

    public static int getInstantiationCt() {
        return instantiateCt;
    }
    public static int getInitializationCt() {
        return initializeCt;
    }
    public static int getExecuteCt() {
        return executeCt;
    }
    public static int getEndCt() {
        return endCt;
    }

    boolean isFinished = false;

    public DummyScheduledCommand() {
        // no requirements for this command
        addRequirements();
        System.out.printf("Instantiating command: class='%s'%n", this.getClass().getName());
        instantiateCt++;
    }

    @Override
    public void initialize() {
        // just print info and increment the initialization ct.
        System.out.printf("%s.initialize()%n", this.getClass().getName());
        initializeCt++;
    }

    @Override
    public void execute() {
        // just print info, increment the execution ct, and reset the finished flag.
        System.out.printf("%s.execute()%n", this.getClass().getName());
        executeCt++;
        isFinished = true;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        // just print info and increment the end count.
        System.out.printf("%s.end()%n", this.getClass().getName());
        endCt++;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
