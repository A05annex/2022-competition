package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TestMotorsSubsystem;

import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.Iterator;


public class TestMotorsCommand extends CommandBase {

    // test speed & time
    public static final double TEST_SPEED = 0.50;
    public static final int TEST_TIME = 100; // in 20 ms increments
    // current motor being tested
    private String currentMotorName;
    // hashmap for motor list
    private final HashMap<String, TalonSRX> motorList = TestMotorsSubsystem.getInstance().getMotorList();
    // iterator that iterates through motors to test
    private Iterator<String> motorItor;

    private boolean isFinished;

    private int currentTime;

    public TestMotorsCommand() {

    }

    @Override
    public void initialize() {
        motorItor = motorList.keySet().iterator();
        currentMotorName = motorItor.next();
        isFinished = false;
        currentTime = 0;
    }

    @Override
    public void execute() {
        printTelemetry();
        currentTime++;
        if (currentTime >= TEST_TIME) {
            currentTime = 0;
            motorList.get(currentMotorName).set(ControlMode.PercentOutput, 0.0);
            if (motorItor.hasNext()) {
                currentMotorName = motorItor.next();
            } else {
                isFinished = true;
            }
        }
        if (!isFinished) {
            testMotorVelocity(motorList.get(currentMotorName), 0.0, currentMotorName);
        }
        // TODO: Implement max speed
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {

    }

//    public static double REAR_SHOOTER_VELOCITY = 17777.0;
    // verify that motors are spinning right
    private void testMotorVelocity(TalonSRX motor, double maxSpeed, String name) {
        motor.set(ControlMode.PercentOutput, TEST_SPEED);
        double velocity = Math.abs(motor.getSelectedSensorVelocity());
        double percentError = velocity / maxSpeed;
        SmartDashboard.putNumber(name, velocity);
        SmartDashboard.putNumber(name + " Velocity", percentError);
        SmartDashboard.putNumber(name + " Current", motor.getStatorCurrent());
    }
    private void printTelemetry() {
        SmartDashboard.putNumber("Time", currentTime);
        SmartDashboard.putString("Current Motor", currentMotorName);
        SmartDashboard.putBoolean("Is Finished?!?!?", isFinished);
    }
}