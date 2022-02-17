package org.a05annex.frc;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public abstract class A05Robot extends TimedRobot {

    Object[] lastLabviewTelemetry = {null, null, null, null, null, null, null, null, null, null};

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
