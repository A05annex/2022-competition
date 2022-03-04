package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoDoubleShootCommandGroup extends SequentialCommandGroup {
    public AutoDoubleShootCommandGroup() {
        super(new AutoShooterBallsCommand(), new CollectorJerkCommand(), new AutoShooterBallsCommand());
    }
}