package org.a05annex.frc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public abstract class A05Robot extends TimedRobot {

    private A05RobotContainer a05RobotContainer;

    Object[] lastLabviewTelemetry = {null, null, null, null, null, null, null, null, null, null};

    protected void setRobotContainer(A05RobotContainer container) {
        a05RobotContainer = container;
    }

    @Override
    public void robotPeriodic() {

        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /**
     * Update telemetry feedback for a real number value. If the value has not changed, no update is sent
     *
     * @param port      (int) The port 0 - 9 to write to.
     * @param key       (String) The key for the telemetry.
     * @param var       (double) The number to be reported.
     */
    @SuppressWarnings("unused")
    protected void labviewTelemetry(int port, String key, double var) {
        if ((lastLabviewTelemetry[port] == null) || (var != (Double)lastLabviewTelemetry[port])) {
            SmartDashboard.putString(String.format("DB/String %d", port), String.format("%s: %10.6f", key, var));
            lastLabviewTelemetry[port] = var;
        }
    }

    /**
     * Update telemetry feedback for an integer value. If the value has not changed, no update is sent
     *
     * @param port      (int) The port 0 - 9 to write to.
     * @param key       (String) The key for the telemetry.
     * @param var       (int) The integer to be reported.
     */
    @SuppressWarnings("unused")
    protected void labviewTelemetry(int port, String key, int var) {
        if ((lastLabviewTelemetry[port] == null) || (var != (Integer)lastLabviewTelemetry[port])) {
            SmartDashboard.putString(String.format("DB/String %d", port), String.format("%s: %d", key, var));
            lastLabviewTelemetry[port] = var;
        }
    }

    /**
     * Update telemetry feedback for a string value. If the value has not changed, no update is sent
     *
     * @param port      (int) The port 0 - 9 to write to.
     * @param key       (String) The key for the telemetry.
     * @param var       (String) The string to be reported.
     */
    @SuppressWarnings("unused")
    protected void labviewTelemetry(int port, String key, String var) {
        if ((lastLabviewTelemetry[port] == null) || ((var != (String)lastLabviewTelemetry[port])) &&
                !var.equals(lastLabviewTelemetry[port])) {
            SmartDashboard.putString(String.format("DB/String %d", port), String.format("%s: %s", key, var));
            lastLabviewTelemetry[port] = var;
        }
    }

    /**
     * Update telemetry feedback for a boolean value. If the value has not changed, no update is sent
     *
     * @param port      (int) The port 0 - 9 to write to.
     * @param key       (String) The key for the telemetry.
     * @param var       (boolean) The boolean to be reported.
     */
    @SuppressWarnings("unused")
    protected void labviewTelemetry(int port, String key, boolean var) {
        if ((lastLabviewTelemetry[port] == null) || (var != (Boolean)lastLabviewTelemetry[port])) {
            SmartDashboard.putString(String.format("DB/String %d", port),
                    String.format("%s: %s", key, var ? "on" : "off"));
            lastLabviewTelemetry[port] = var;
        }
    }

    /**
     * Initialize the labview telemetry dashboard to empty entries.
     */
    protected void initLabviewTelemetry() {
        for (int i = 0; i < 10; i++) {
            SmartDashboard.putString(String.format("DB/String %d", i), " ");
        }

    }

}
