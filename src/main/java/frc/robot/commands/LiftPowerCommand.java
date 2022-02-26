package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.LiftSubsystem;


public class LiftPowerCommand extends CommandBase {
    private final LiftSubsystem m_liftSubsystem = LiftSubsystem.getInstance();
    private final JoystickButton m_leftUpButton;
    private final JoystickButton m_leftDownButton;
    private final JoystickButton m_rightUpButton;
    private final JoystickButton m_rightDownButton;

    public LiftPowerCommand(JoystickButton leftUpButton, JoystickButton leftDownButton,
                            JoystickButton rightUpButton, JoystickButton rightDownButton) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_liftSubsystem);
        m_leftUpButton = leftUpButton;
        m_leftDownButton = leftDownButton;
        m_rightUpButton = rightUpButton;
        m_rightDownButton = rightDownButton;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (m_leftUpButton.get()) {
            m_liftSubsystem.setLeftPower(-LiftSubsystem.LIFT_POWER);
        } else if (m_leftDownButton.get()) {
            m_liftSubsystem.setLeftPower(LiftSubsystem.LIFT_POWER);
        } else {
            m_liftSubsystem.setLeftPower(0.0);
        }

        if (m_rightUpButton.get()) {
            m_liftSubsystem.setRightPower(-LiftSubsystem.LIFT_POWER);
        } else if (m_rightDownButton.get()) {
            m_liftSubsystem.setRightPower(LiftSubsystem.LIFT_POWER);
        } else {
            m_liftSubsystem.setRightPower(0.0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_liftSubsystem.setLeftPower(0.0);
        m_liftSubsystem.setRightPower(0.0);
    }
}
