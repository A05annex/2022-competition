// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;


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
    ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();
    CollectorSubsystem m_collectorSubsystem = CollectorSubsystem.getInstance();
    FeederSubsystem m_feederSubsystem = FeederSubsystem.getInstance();
    LiftSubsystem m_liftSubsystem = LiftSubsystem.getInstance();

    // command declarations
    DriveCommand m_driveCommand;
    CollectorHoldCommand m_collectorCommand;
    FeederCommand m_feederCommand;
    LiftManualCommand m_liftManualCommand;
    private final Command autoCommand = null; // autonomous command

    // declare NavX, used for resetting initial heading
    NavX m_navx = NavX.getInstance();

    // controller declarations
    XboxController m_xbox = new XboxController(Constants.XBOX_PORT);
    XboxController m_hang_xbox = new XboxController(Constants.HANG_XBOX_PORT);

    // controller button declarations
    JoystickButton m_xboxA = new JoystickButton(m_xbox, 1);
    JoystickButton m_xboxB = new JoystickButton(m_xbox, 2);
    JoystickButton m_xboxX = new JoystickButton(m_xbox, 3);
    JoystickButton m_xboxY = new JoystickButton(m_xbox, 4);
    JoystickButton m_xboxLeftBumper = new JoystickButton(m_xbox, 5);
    JoystickButton m_xboxRightBumper = new JoystickButton(m_xbox, 6);
    JoystickButton m_xboxBack = new JoystickButton(m_xbox, 7);
    JoystickButton m_xboxStart = new JoystickButton(m_xbox, 8);
    JoystickButton m_xboxLeftStickPress = new JoystickButton(m_xbox, 9);
    JoystickButton m_xboxRightStickPress = new JoystickButton(m_xbox, 10);

    // hang xbox controller buttons
    JoystickButton m_hangA = new JoystickButton(m_hang_xbox, 1);
    JoystickButton m_hangB = new JoystickButton(m_hang_xbox, 2);
    JoystickButton m_hangX = new JoystickButton(m_hang_xbox, 3);
    JoystickButton m_hangY = new JoystickButton(m_hang_xbox, 4);


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        // commands
        m_driveCommand = new DriveCommand(m_xbox, m_xboxLeftBumper); // uses both sticks and LB for limelight targeting
        m_collectorCommand = new CollectorHoldCommand(m_xboxRightBumper);
        m_feederCommand = new FeederCommand(m_xboxLeftStickPress, m_xboxRightStickPress);
        m_liftManualCommand = new LiftManualCommand(m_hangY, m_hangB, m_hangX, m_hangA);

        // set default commands
        m_driveSubsystem.setDefaultCommand(m_driveCommand);
        m_collectorSubsystem.setDefaultCommand(m_collectorCommand);
        m_feederSubsystem.setDefaultCommand(m_feederCommand);
        m_liftSubsystem.setDefaultCommand(m_liftManualCommand);

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
        m_xboxBack.whenPressed(new InstantCommand(m_navx::initializeHeadingAndNav)); // Reset the NavX field relativity
        m_xboxX.whenPressed(new CollectorJerkCommand());
        m_xboxB.whenPressed(new AutoShooterBallsCommand());
        m_xboxA.whenPressed(new AutoShooterStartCommand());
        m_xboxY.whenPressed(new ShooterSetSpeedCommand());
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
