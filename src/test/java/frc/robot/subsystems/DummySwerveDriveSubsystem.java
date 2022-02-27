package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleD;

public class DummySwerveDriveSubsystem extends SubsystemBase implements ISwerveDrive {

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this DummySwerveDriveSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static DummySwerveDriveSubsystem INSTANCE = new DummySwerveDriveSubsystem();

    /**
     * Returns the Singleton instance of this DummySwerveDriveSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code DummySwerveDriveSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static DummySwerveDriveSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this DummySwerveDriveSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private DummySwerveDriveSubsystem() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }

    @Override
    public void setFieldPosition(double fieldX, double fieldY, AngleD heading) {
        System.out.printf("setFieldPosition:                      %10.3f %10.3f %10.3f%n",
                fieldX, fieldY, heading);
    }

    @Override
    public void swerveDriveComponents(double forward, double strafe, double rotation) {
        System.out.printf("swerveDriveComponents:      %d %10.3f %10.3f %10.3f%n", System.currentTimeMillis(),
                forward, strafe, rotation);
    }

    @Override
    public void prepareForDriveComponents(double forward, double strafe, double rotation) {
        System.out.printf("prepareForDriveComponents:  %d %10.3f %10.3f %10.3f%n", System.currentTimeMillis(),
                forward, strafe, rotation);
    }

    @Override
    public void swerveDrive(AngleConstantD chassisDirection, double speed, double rotation) {
        System.out.printf("swerveDrive:  %d %10.3f %10.3f %10.3f%n", System.currentTimeMillis(),
                chassisDirection.getRadians(), speed, rotation);
    }

    @Override
    public void swerveDriveFieldRelative(AngleConstantD fieldDirection, double speed, double rotation) {
        System.out.printf("swerveDriveFieldRelative:  %d %10.3f %10.3f %10.3f%n", System.currentTimeMillis(),
                fieldDirection.getRadians(), speed, rotation);
    }

    @Override
    public void setHeading(AngleConstantD targetHeading) {
        System.out.printf("setFieldHeading:        %10.3f%n", targetHeading.getRadians());
    }
}

