package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class AutoStartCommandGroup extends ParallelCommandGroup {
    public AutoStartCommandGroup() {
        super(new AutoShooterStartCommand(), new AutoDropCollectorCommand());
    }
}