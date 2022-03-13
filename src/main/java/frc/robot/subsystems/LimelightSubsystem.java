package frc.robot.subsystems;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.a05annex.util.AngleConstantD;
import org.a05annex.util.AngleUnit;
import frc.robot.Constants.LimelightCalibrationPoint;

public class LimelightSubsystem extends SubsystemBase {
    /**
     * The Singleton instance of this LimelightSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private static LimelightSubsystem INSTANCE;

    /**
     * Returns the Singleton instance of this LimelightSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code LimelightSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static LimelightSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new LimelightSubsystem();
        }
        return INSTANCE;
    }

    // pipeline constants
    public static final class Pipelines {
        public static final int
                DRIVER = 1,
                SHOOTER = 4;
    }

    // get limelight NetworkTable
    private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight");

    // set pipeline to default initially
    private int m_pipeline = Pipelines.SHOOTER;

    // data class
    private static class TargetData {
        double tv;
        double tx;
        double ty;
        double ta;
        double ts;

        TargetData(double tv, double tx, double ty, double ta, double ts) {
            this.tv = tv;
            this.tx = tx;
            this.ty = ty;
            this.ta = ta;
            this.ts = ts;
        }
    }

    /**
     * Creates a new instance of this LimelightSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private LimelightSubsystem() {
        // set default pipeline
        setPipeline(m_pipeline);
    }

    public int getPipeline() {
        return m_pipeline;
    }

    public void setPipeline(int pipeline) {
        m_pipeline = pipeline;
        m_table.getEntry("pipeline").setNumber(m_pipeline);
    }

    /**
     *
     * @return (AngleConstantD) The difference between the target and the limelight cursor, in degrees.
     */
    public AngleConstantD getTargetError() {
        double tx = getTargetData().tx;
        if (tx == -100.0) {
            // no limelight, so no error
            return new AngleConstantD(AngleConstantD.ZERO);
        }
        return new AngleConstantD(AngleUnit.DEGREES, getTargetData().tx);
    }

    public double distanceToTarget() {
        double ty = getTargetData().ty;
        if (ty == -100.0) {
            return 0.0;
        }
        return ((Constants.TARGET_HEIGHT - Constants.LIMELIGHT_HEIGHT) /
                Math.tan(Math.toRadians(Constants.LIMELIGHT_ANGLE + ty))) + Constants.TARGET_RADIUS;
    }

    public enum CAN_SHOOT {
        YES("yes"),
        TOO_CLOSE("too close"),
        TOO_FAR("too far"),
        NO_TARGET("no target"),
        NO_LIMELIGHT("no limelight");

        private final String name;

        CAN_SHOOT(String name) {
            this.name = name;
        }
    }

    public CAN_SHOOT canShoot() {
        double ty = getTargetData().ty;
        double tx = getTargetData().tx;
        double tv = getTargetData().tv;
        LimelightCalibrationPoint[] limelightPoints = Constants.LIMELIGHT_CALIBRATION_POINTS;

        // networktables returns default value
        if (ty == -100.0) {
            return CAN_SHOOT.NO_LIMELIGHT;
        }

        // no target
        if (tv == 0.0) {
            return CAN_SHOOT.NO_TARGET;
        }

        // adjust ty using tx if we are off
        ty = ty / Math.cos(Math.toRadians(tx));

        // closer than first cal point
        if (ty > limelightPoints[0].ty) {
            return CAN_SHOOT.TOO_CLOSE;
        }

        // further than last cal point
        if (ty < limelightPoints[limelightPoints.length - 1].ty) {
            return CAN_SHOOT.TOO_FAR;
        }

        return CAN_SHOOT.YES;
    }

    public LimelightCalibrationPoint getShooterSpeeds() {
        double ty = getTargetData().ty;
        LimelightCalibrationPoint[] limelightPoints = Constants.LIMELIGHT_CALIBRATION_POINTS;

        if (canShoot() != CAN_SHOOT.YES) {
            return null;
        }

        // iterate through points and find the point we are inside
        int closePoint = 0;
        int farPoint = 1;
        while (farPoint < limelightPoints.length) {
            if (ty > limelightPoints[farPoint].ty) {
                // find parametric distance from close point (0 to 1)
                double intervalAngle = limelightPoints[farPoint].ty - limelightPoints[closePoint].ty;
                double deltaAngle = ty - limelightPoints[closePoint].ty;
                double parametricDistance = deltaAngle / intervalAngle;

                // find speeds using parametric distance
                double frontSpeed = (limelightPoints[closePoint].frontSpeed * (1-parametricDistance)) +
                        (limelightPoints[farPoint].frontSpeed * parametricDistance);
                double rearSpeed = (limelightPoints[closePoint].rearSpeed * (1-parametricDistance)) +
                        (limelightPoints[farPoint].rearSpeed * parametricDistance);

                return new LimelightCalibrationPoint(ty, frontSpeed, rearSpeed);
            } else {
                // wrong point, keep going
                closePoint++;
                farPoint++;
            }
        }
        // something went wrong
        return null;
    }

    /**
     * Returns the data class to hold all target data from the limelight.
     * If there is no data, all values will be -1.0.
     *
     * tv (double): Whether the limelight has any valid targets (0 or 1)
     * tx (double): Horizontal offset from crosshair to target (-29.8 degrees to 29.8 degrees)
     * ty (double): Vertical offset from crosshair to target (-24.85 degrees to 24.85 degrees)
     * ta (double): Target area (0% of image to 100% of image)
     * ts (double): Target skew or rotation (-90 degrees to 0 degrees)
     */
    public TargetData getTargetData() {
        return new TargetData(
                m_table.getEntry("tv").getDouble(-100.0),
                m_table.getEntry("tx").getDouble(-100.0),
                m_table.getEntry("ty").getDouble(-100.0),
                m_table.getEntry("ta").getDouble(-100.0),
                m_table.getEntry("ts").getDouble(-100.0)
        );
    }

    // smart dashboard methods
    public void printTargetData() {
        TargetData data = getTargetData();
        SmartDashboard.putNumber("tv", data.tv);
        SmartDashboard.putNumber("tx", data.tx);
        SmartDashboard.putNumber("ty", data.ty);
        SmartDashboard.putNumber("ta", data.ta);
        SmartDashboard.putNumber("ts", data.ts);
    }

    public void printXY() {
        TargetData data = getTargetData();
        SmartDashboard.putNumber("tx", data.tx);
        SmartDashboard.putNumber("ty", data.ty);
    }

    public void printCanShoot() {
        SmartDashboard.putString("shoot status", canShoot().name);
    }

    public void printCanShootBool() {
        SmartDashboard.putBoolean("can shoot?", canShoot() == CAN_SHOOT.YES);
    }

    public void printShooterPowers() {
        LimelightCalibrationPoint powers = getShooterSpeeds();
        if (powers != null) {
            SmartDashboard.putNumber("front limelight", powers.frontSpeed);
            SmartDashboard.putNumber("rear limelight", powers.rearSpeed);
        } else {
            SmartDashboard.putNumber("front limelight", 0.0);
            SmartDashboard.putNumber("rear limelight", 0.0);
        }
    }
}

