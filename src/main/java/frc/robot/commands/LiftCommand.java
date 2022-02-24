package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.LiftSubsystem;


public class LiftCommand extends CommandBase {
    private final LiftSubsystem m_liftSubsystem = LiftSubsystem.getInstance();
    private final JoystickButton m_startButton;
    private final JoystickButton m_cancelButton;
    private int m_state = 0;

    public LiftCommand(JoystickButton startButton, JoystickButton cancelButton) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_liftSubsystem);
        m_startButton = startButton;
        m_cancelButton = cancelButton;
    }

    @Override
    public void initialize() {
        m_state = 0;
        // when scheduled, ready to hang: lift hangers
        m_liftSubsystem.setRightPosition(LiftSubsystem.HangerPositions.RIGHT_UP);
        m_liftSubsystem.setLeftPosition(LiftSubsystem.HangerPositions.LEFT_UP);
    }

    @Override
    public void execute() {
        // press startButton when ready to start auto hang
        if (m_startButton.get()) {
            m_state = 1;
        }

        // start auto hang: lower right hanger onto bar
        if (m_state == 1) {
            m_liftSubsystem.setRightPosition(LiftSubsystem.HangerPositions.RIGHT_DOWN);
            if (m_liftSubsystem.getRightPosition() < LiftSubsystem.HangerPositions.RIGHT_DOWN +
                    LiftSubsystem.LIFT_TOLERANCE) { // lift is close enough to the set position
                m_state = 2;
            }
        }

        // robot is ready to grab high bar
        if (m_state == 2) {
            m_liftSubsystem.setLeftPosition(LiftSubsystem.HangerPositions.LEFT_DOWN);
            if (m_liftSubsystem.getLeftPosition() < LiftSubsystem.HangerPositions.LEFT_DOWN +
                    LiftSubsystem.LIFT_TOLERANCE) {
                m_state = 3;
            }
        }

        // robot has grabbed high bar, lift the right hanger back up
        if (m_state == 3) {
            m_liftSubsystem.setRightPosition(LiftSubsystem.HangerPositions.RIGHT_UP);
            // done
        }
    }

    @Override
    public boolean isFinished() {
        return m_cancelButton.get(); // if cancel button pressed, end immediately and lift hooks
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) { // this means the cancel button was pressed
            m_liftSubsystem.setRightPosition(LiftSubsystem.HangerPositions.RIGHT_UP);
            m_liftSubsystem.setLeftPosition(LiftSubsystem.HangerPositions.LEFT_UP);
        }
    }
}
