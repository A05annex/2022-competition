package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestMotorsSubsystem;

import java.util.ArrayList;
import java.util.Iterator;


public class TestMotorsCommand extends CommandBase {

    // test speed
    public static double TEST_SPEED = 0.50;
    // current motor being tested
    private TalonSRX currentMotor;
    // iterator that iterates through motors to test
    private Iterator<TalonSRX> motorItor = TestMotorsSubsystem.getInstance().getMotorList().iterator();
    // iterator that iterates through motors to test

    public TestMotorsCommand() {

    }

    @Override
    public void initialize() {
        currentMotor = motorItor.next();
    }

    @Override
    public void execute() {
        testMotorVelocity(currentMotor, 0.0, "Current");
        // TODO: Implement different names
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }

//    public static double REAR_SHOOTER_VELOCITY = 17777.0;
    // verify that motors are spinning right
    public static void testMotorVelocity(TalonSRX motor, double maxSpeed, String name) {
        motor.set(ControlMode.PercentOutput, TEST_SPEED);
        double velocity = Math.abs(motor.getSelectedSensorVelocity());
        double percentError = velocity / maxSpeed;
        SmartDashboard.putNumber(name, velocity);
        SmartDashboard.putNumber(name + " Velocity", percentError);
    }

}
