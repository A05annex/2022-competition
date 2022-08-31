package org.a05annex.frc;

import frc.robot.commands.DriveCommand;
import org.a05annex.frc.commands.A05DriveCommand;
import org.a05annex.frc.subsystems.DriveSubsystem;

public abstract class A05RobotContainer
{

    // declare NavX, used for resetting initial heading
    protected NavX m_navx = NavX.getInstance();

    protected DriveSubsystem m_driveSubsystem = DriveSubsystem.getInstance();
    protected A05DriveCommand m_driveCommand;

    public A05RobotContainer() {
        m_driveCommand = new A05DriveCommand(m_xbox);
    }
}
