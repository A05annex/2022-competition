package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class DummyStopAndRunCommand extends CommandBase {
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

    final private long startTime = System.currentTimeMillis();
    final private long endTime = startTime + 2000;

    public DummyStopAndRunCommand() {
        addRequirements();
        System.out.printf("Instantiating command: class='%s'%n", this.getClass().getName());
        System.out.printf("          '%s':  ends at %d%n", this.getClass().getName(), endTime);
        instantiateCt++;
    }

    @Override
    public void initialize() {
        initializeCt++;
    }

    @Override
    public void execute() {
        executeCt++;
        System.out.print(".");
        System.out.flush();
    }

    @Override
    public boolean isFinished() {
        // Return true once the finish time is reached.
        return (System.currentTimeMillis() > endTime);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.printf("%n          '%s':  ends after %.3f%n", this.getClass().getName(),
                (endTime-startTime)/1000.0);
        endCt++;
    }
}
