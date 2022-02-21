package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ShooterSubsystem;


public class ShooterCommand extends CommandBase {

    private final ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();
    private final JoystickButton m_toggleButton;

    private boolean m_isRunning = false;
    private boolean m_lastToggleButton = false;

    /**
     * Runs the shooter at the constant speeds in ShooterSubsystem.
     * @param toggleButton Toggles the shooter on/off.
     */
    public ShooterCommand(JoystickButton toggleButton) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_shooterSubsystem);
        m_toggleButton = toggleButton;
    }

    @Override
    public void initialize() {
        m_isRunning = false;
        m_lastToggleButton = false;
    }

    @Override
    public void execute() {
        if (m_toggleButton.get() && !m_lastToggleButton) {
            m_isRunning = !m_isRunning;
        }

        if (m_isRunning) {
            m_shooterSubsystem.setFrontShooter(m_shooterSubsystem.frontShooterTestSpeed);
            m_shooterSubsystem.setRearShooter(m_shooterSubsystem.rearShooterTestSpeed);
        } else {
            m_shooterSubsystem.setFrontShooter(0.0);
            m_shooterSubsystem.setRearShooter(0.0);
        }

        m_lastToggleButton = m_toggleButton.get();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.setFrontShooter(0.0);
        m_shooterSubsystem.setRearShooter(0.0);
    }
}
