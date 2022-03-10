package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DummySwerveDriveSubsystem;
import org.a05annex.util.geo2d.KochanekBartelsSpline;
import org.jetbrains.annotations.NotNull;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.platform.runner.JUnitPlatform;
import org.junit.runner.RunWith;

import java.io.File;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** This is a test of the {@link frc.robot.commands.AutonomousPathCommand} that uses a test path
 * with both scheduled commands and stop-and-run commands. The test path is a 5 control point path. Stop-and-run
 * commands happen at the 1st, 3rd, and 5th (last) control points, scheduled commands happens at 2
 * locations on the path.
 */
@RunWith(JUnitPlatform.class)
public class TestAutonomousPathCommand {

    /**
     * This is extension of the {@link AutonomousPathCommand} that overrides the
     * {@link Command#runsWhenDisabled()} to return {@code true} so we can run in
     * the test environment which believes the robot is disabled.
     */
    static class TestAutonomousPathCommandWrapper extends AutonomousPathCommand {

        /**
         * Instantiate the {@code AutonomousPathCommand}.
         *
         * @param path                   The path description.
         * @param driveSubsystem         The swerve drive subsystem.
         * @param additionalRequirements Additional required subsystems.
         */
        public TestAutonomousPathCommandWrapper(@NotNull KochanekBartelsSpline path,
                                                @NotNull Subsystem driveSubsystem,
                                                Subsystem... additionalRequirements) {
            super(path, driveSubsystem, additionalRequirements);
        }

        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }


    String testPathName = "./src/test/resources/paths/AutonomousPathCommandTest.json";

    /**
     *
     */
    @Test
    @DisplayName("Test AutonomousPathCommand")
    void test_autonomousPathCommand() {
        // instantiate the AutonomousPathCommand with the test path and the DummySwerveDriveSubsystem,
        // get a scheduler and schedule the Autonomous
        File file = new File(testPathName);
        System.out.println(file.getAbsolutePath());
        KochanekBartelsSpline path = new KochanekBartelsSpline();
        assertTrue(path.loadPath(testPathName));

        DummyScheduledCommand.resetCounts();
        DummyStopAndRunCommand.resetCounts();
        TestAutonomousPathCommandWrapper autonomousPathCommend = new TestAutonomousPathCommandWrapper(
                path, DummySwerveDriveSubsystem.getInstance());

        // Run the command using the scheduler so we can test that things are actually scheduled correctly.
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.schedule(autonomousPathCommend);
        scheduler.run();
        long nextRunTime = System.currentTimeMillis();
        while (scheduler.isScheduled(autonomousPathCommend)) {
            scheduler.run();
            nextRunTime += 20L;
            long sleepTime = nextRunTime - System.currentTimeMillis();
            try {
                if (sleepTime > 0L) {
                    //noinspection BusyWait
                    Thread.sleep(nextRunTime - System.currentTimeMillis());
                }
            } catch (InterruptedException e) {
                break;
            }
        }

        // The path has been run, assure the DummyScheduledCommand was instantiated and executed twice
        assertEquals(2, DummyScheduledCommand.getInstantiationCt());
        assertEquals(2, DummyScheduledCommand.getInitializationCt());
        assertEquals(2, DummyScheduledCommand.getExecuteCt());
        assertEquals(2, DummyScheduledCommand.getEndCt());

        // Assure the stop-and-run was actually run 3 times. with about 100.0 executes (2 seconds) fo each run.
        assertEquals(3, DummyStopAndRunCommand.getInstantiationCt());
        assertEquals(3, DummyStopAndRunCommand.getInitializationCt());
        assertEquals(303.0, DummyStopAndRunCommand.getExecuteCt(),3.0);
        assertEquals(3, DummyStopAndRunCommand.getEndCt());

        // Assure the stop-and-run duration is about 6 second (6000 milliseconds). It is expected that each command
        // can overrun about 1 cycle, and that cycle times vary to 25ms max.
        assertEquals(6040.0, (double)autonomousPathCommend.stopAndRunDuration,40.0);
    }
}
