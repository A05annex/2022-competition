// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.LiftTestCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LiftSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
    // subsystem declarations (should all be instances)
    DriveSubsystem m_driveSubsystem = DriveSubsystem.getInstance();
    LiftSubsystem m_liftSubsystem = LiftSubsystem.getInstance();

    // command declarations
    DriveCommand m_driveCommand;
    LiftTestCommand m_liftTestCommand;
    private final Command autoCommand = null; // autonomous command

    // declare NavX, used for resetting initial heading
    NavX m_navx = NavX.getInstance();

    // controller declarations
    XboxController m_xbox = new XboxController(Constants.XBOX_PORT);

    // controller button declarations
    JoystickButton m_xboxX = new JoystickButton(m_xbox, 0);
    JoystickButton m_xboxA = new JoystickButton(m_xbox, 1);
    JoystickButton m_xboxB = new JoystickButton(m_xbox, 2);
    JoystickButton m_xboxY = new JoystickButton(m_xbox, 3);



    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        // commands
        m_driveCommand = new DriveCommand(m_xbox);

        // set default commands
        //m_driveSubsystem.setDefaultCommand(m_driveCommand);
        m_liftSubsystem.setDefaultCommand(m_liftTestCommand);

        // Configure the button bindings
        configureButtonBindings();
    }
    
    
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings()
    {
        // Add button to command mappings here.
        // See https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html
        //m_xboxA.whenPressed(new InstantCommand(m_navx::initializeHeadingAndNav));
        m_xboxA.whenPressed(new InstantCommand(m_liftSubsystem::incLeftLiftPosition));
        m_xboxB.whenPressed(new InstantCommand(m_liftSubsystem::incRightLiftPosition));
        m_xboxX.whenPressed(new InstantCommand(m_liftSubsystem::decLeftLiftPosition));
        m_xboxY.whenPressed(new InstantCommand(m_liftSubsystem::decRightLiftPosition));
    }
    

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // An ExampleCommand will run in autonomous
        return autoCommand;
    }
}
