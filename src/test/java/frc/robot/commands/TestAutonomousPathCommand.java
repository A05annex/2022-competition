package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.NavX;
import frc.robot.subsystems.DummySwerveDriveSubsystem;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;
import org.a05annex.util.geo2d.KochanekBartelsSpline;
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
        Boolean exists = file.exists();
        KochanekBartelsSpline path = new KochanekBartelsSpline();
        assertTrue(path.loadPath(testPathName));

        AutonomousPathCommand autonomousPathCommend = new AutonomousPathCommand(
                path,DummySwerveDriveSubsystem.getInstance());
        autonomousPathCommend.initialize();

        long startTime = System.currentTimeMillis();
        System.out.println(String.format("Start time: %d", startTime));
        long nextTime = startTime + 20;
        while (!autonomousPathCommend.isFinished()) {
            autonomousPathCommend.execute();
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                break;
            }
        }
        autonomousPathCommend.end(false);
    }
}
