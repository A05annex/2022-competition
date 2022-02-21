package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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
    private double m_collectorPower = -0.7;

    /**
     * Creates a new instance of this CollectorSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private CollectorSubsystem() {
        m_collector.configFactoryDefault();
        m_collector.setNeutralMode(NeutralMode.Coast); // coast mode, may want to change this
    }

    /**
     * Sets the power of the collector. Must be called every loop.
     * @param power (double) Power to set the collector to, from -1.0 to 1.0.
     */
    public void setPower(double power) {
        m_collector.set(ControlMode.PercentOutput, power);
    }

    public void updateCollectorPower() {
        m_collectorPower = Constants.updateConstant("Collector Power", m_collectorPower,
                -1.0, 1.0);
    }

    public double getCollectorPower() {
        return m_collectorPower;
    }

}

