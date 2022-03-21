package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CollectorSubsystem extends SubsystemBase {
    /**
     * The Singleton instance of this CollectorSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private static CollectorSubsystem INSTANCE;

    /**
     * Returns the Singleton instance of this CollectorSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code CollectorSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static CollectorSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new CollectorSubsystem();
        }
        return INSTANCE;
    }

    private final TalonSRX m_collector = new TalonSRX(Constants.CAN_Devices.COLLECTOR);
    private double m_lastPower;

    // power to collect
    public static double COLLECTOR_POWER = -0.6;

    // power to jerk
    public static double BACK_POWER = -0.75;
    public static double FORWARD_POWER = -0.75;

    // number of 20ms cycles to jerk collector backwards and forwards
    public static final int BACK_CYCLES = 4; // 80 ms
    public static final int FORWARD_CYCLES = 7; // 100 ms
    public static final int STOP_CYCLES = 10; // 200 ms

    public static double MAX_SPEED = 0.0; // TODO

    /**
     * Creates a new instance of this CollectorSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private CollectorSubsystem() {
        m_collector.configFactoryDefault();
        m_collector.setNeutralMode(NeutralMode.Brake);
        m_lastPower = 0.0;

        // encoder
        m_collector.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        m_collector.setSensorPhase(false);
    }

    /**
     * Sets the power of the collector. Must be called every loop.
     * @param power (double) Power to set the collector to, from -1.0 to 1.0.
     */
    public void setPower(double power) {
        m_collector.set(ControlMode.PercentOutput, power);
        m_lastPower = power;
    }

    /**
     * Sets the speed of the collector using the encoder.
     * @param power (double) Power to set the collector to, from -1.0 to 1.0.
     */
    public void setSpeed(double power) {
        m_lastPower = power;
        double speed = MAX_SPEED * power;
        m_collector.set(ControlMode.Velocity, speed);
    }

    public double getSpeed() {
        return m_collector.getSelectedSensorVelocity();
    }

    public void updateCollectorPower() {
        COLLECTOR_POWER = Constants.updateConstant("Collector Power", COLLECTOR_POWER,
                -1.0, 1.0);
    }

    public double getPower() {
        return m_lastPower;
    }

    public void printSpeed() {
        SmartDashboard.putNumber("collector enc", getSpeed());
    }

}
