package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.NavX;
import frc.robot.subsystems.ISwerveDrive;
import org.a05annex.util.geo2d.KochanekBartelsSpline;

/**
 * This is a command that follows an autonomous path created in the
 * <a href="https://github.com/A05annex/SwervePathPlanning">Swerve Path Planning</a> app. The key capabilities
 * of this command is that it acts as a dynamic command group that orchestrates the robot path and:
 * <ul>
 *     <li>launches other commands that happen concurrently with path following</li>
 *     <li>stops the robot on the path, initiates a command (like aiming and shooting), and continues following
 *     the path when the initiated command completes.</li>
 * </ul>
 * Note that when run in a test environment the navx device is instantiated as a simulation device rather than a
 * real physical devise.
 */
public class AutonomousPathCommand extends CommandBase {

    private final ISwerveDrive swerveDrive;
    private final KochanekBartelsSpline spline;
    private KochanekBartelsSpline.PathFollower pathFollower;
    protected KochanekBartelsSpline.PathPoint pathPoint = null;
    private boolean isFinished = false;
    private long startTime;
    private long stopAndRunStartTime = 0;
    private long stopAndRunDuration = 0;
    private Command stopAndRunCommand = null;

    public AutonomousPathCommand(KochanekBartelsSpline path, Subsystem driveSubsystem) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(driveSubsystem);
        swerveDrive = (ISwerveDrive)driveSubsystem;
        spline = path;
        pathFollower = spline.getPathFollower();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        pathFollower = spline.getPathFollower();
        isFinished = false;

        initializeRobotForPath();
    }
    /**
     * Initialize the robot to run this path. This initialization consists specifically of
     * <ul>
     * <li>making sure the NavX is aware of robot heading prior to starting along the path</li>
     * <li>assuring The serve modules are rotated to the correct orientation for the first
     * expected Forward, strafe, and rotate components that will be set for the path (eliminating
     * drift while the robot is trying to get all the modules to the correct orientation.</li>
     * </ul>
     */
    public void initializeRobotForPath() {
        pathPoint = pathFollower.getPointAt(0.0);
        if (pathPoint != null) {
            NavX.getInstance().initializeHeadingAndNav(pathPoint.fieldHeading);
            double forward = pathPoint.speedForward / Constants.MAX_METERS_PER_SEC;
            double strafe = pathPoint.speedStrafe / Constants.MAX_METERS_PER_SEC;
            double rotation = (pathPoint.speedRotation / Constants.MAX_RADIANS_PER_SEC);
            swerveDrive.prepareForDriveComponents(forward, strafe, rotation);
            startTime = System.currentTimeMillis();
            if ((null != pathPoint.action) && (null != pathPoint.action.command) &&
                    (KochanekBartelsSpline.RobotActionType.STOP_AND_RUN_COMMAND == pathPoint.action.actionType)) {
                String commandClass = "frc.robot.commands." + pathPoint.action.command;
                Command command = null;
                Object obj;
                try {
                    obj = Class.forName(commandClass).newInstance();
                    if (obj instanceof Command) {
                        command = (Command)obj;
                    }
                    stopAndRunCommand = command;
                    stopAndRunStartTime = System.currentTimeMillis();
                    stopAndRunCommand.initialize();
                } catch (final Exception t) {
                    System.out.println(
                            String.format("Could not instantiate command: class='%s'; continuing with path.",
                                    commandClass));
                }
            }
        }

    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        if (null != stopAndRunCommand) {
            // There is an active stop-and-run command
            stopAndRunCommand.execute();
            return;
        }
        double pathTime = (System.currentTimeMillis() - startTime - stopAndRunDuration) / 1000.0;
        pathPoint = pathFollower.getPointAt(pathTime);
        if (pathPoint == null) {
            isFinished = true;
            swerveDrive.swerveDriveComponents(0, 0, 0);
        } else {
            // for 2022 Rapid React we have added scheduled actions and stop-and-run actions. This makes this
            // command very much like a wpilib CommandGroup action. The interesting thing about this action
            // that it gets all its sequencing from the path file - which was built without access to the
            // actual code and commands that may be scheduled or stop_and_run. These commands are instantiated by
            // reflection, so only the name of the command is required during path planning.
            if ((null != pathPoint.action) && (null != pathPoint.action.command)) {
                // OK, the new thing - an action - first, let's see if we can instantiate the command for that
                // action. This is the same code for either a scheduled or stop-and-run action. If is required
                // that any specified action is in the 'frc.robot.commands' package, and has a no argument
                // constructor. Additionally, if it is a scheduled command, it cannot require the drive subsystem
                // or path following would be interrupted.
                String commandClass = "frc.robot.commands." + pathPoint.action.command;
                Command command = null;
                //System.out.println(String.format("Instantiating command: class='%s'", commandClass));
                final Object obj;
                try {
                    // instantiate the command (no argument constructor required)
                    obj = Class.forName(commandClass).newInstance();
                    if (obj instanceof Command) {
                        command = (Command)obj;
                    }
                    // OK, we've instantiated the command, now either schedule it, or run it.
                    if (KochanekBartelsSpline.RobotActionType.SCHEDULE_COMMAND == pathPoint.action.actionType) {
                        // this one is really simple - we just schedule the command, and it happens in
                        // parallel with path following.
                        CommandScheduler.getInstance().schedule(command);
                    } else if (KochanekBartelsSpline.RobotActionType.STOP_AND_RUN_COMMAND == pathPoint.action.actionType) {
                        // this is a bit more complicated, we are going to run the command inside this command,
                        // then resume path following when this command completes.
                        stopAndRunCommand = command;
                        swerveDrive.swerveDriveComponents(0, 0, 0);
                        stopAndRunStartTime = System.currentTimeMillis();
                        stopAndRunCommand.initialize();
                        return;
                    }
                } catch (final Exception t) {
                    System.out.println(
                            String.format("Could not instantiate command: class='%s'; continuing with path.",
                                    commandClass));
                }

            }

            double errorRotation = (pathPoint.fieldHeading.getRadians() - NavX.getInstance().getHeading().getRadians()) *
                    Constants.DRIVE_ORIENTATION_kP;
            // The expected heading is included in the PathPoint. The path point is the instantaneous
            // speed and position that we want to be at NOW. If the heading is incorrect, then the
            // direction the forward and strafe is incorrect and we will be at the wrong place on
            // the field. So we need a PID correction of heading incorporated here.
            double forward = pathPoint.speedForward / Constants.MAX_METERS_PER_SEC;
            double strafe = pathPoint.speedStrafe / Constants.MAX_METERS_PER_SEC;
            double rotation = (pathPoint.speedRotation / Constants.MAX_RADIANS_PER_SEC) + errorRotation;
//            double rotation = (point.speedRotation / Constants.MAX_RADIANS_PER_SEC);
            swerveDrive.swerveDriveComponents(forward, strafe, rotation);
            NavX.getInstance().setExpectedHeadingToCurrent();
        }

    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        if (null != stopAndRunCommand) {
            if (stopAndRunCommand.isFinished()) {
                // done with the stop and run, so end it and increment the stop and run duration.
                stopAndRunCommand.end(false);
                long duration = System.currentTimeMillis() - stopAndRunStartTime;
                stopAndRunDuration += duration;
                stopAndRunCommand = null;
                stopAndRunStartTime = 0;
                // I'm going to assume that if we stop to do something it may involve rotation to aim
                // for shooting, but, probably does not involve any translation.
                // TODO - what happens if the robot has moved? The path will not continue
                //  correctly unless we move back to the start point.
            }
        }
        return isFinished;
    }

    /**
     * The action to take when the command ends - in this case,  if there is a stop-and-run command active,
     * we end it, and then we stop the robot drive.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        if (interrupted && (null != stopAndRunCommand)) {
            stopAndRunCommand.end(true);
        }
        swerveDrive.swerveDriveComponents(0, 0, 0);
    }
}
