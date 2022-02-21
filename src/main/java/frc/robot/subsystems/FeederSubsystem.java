package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {
    /**
     * The Singleton instance of this FeederSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private static FeederSubsystem INSTANCE;

    /**
     * Returns the Singleton instance of this FeederSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code FeederSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static FeederSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new FeederSubsystem();
        }
        return INSTANCE;
    }

    private final TalonSRX m_feeder = new TalonSRX(Constants.CAN_Devices.FEEDER);
    public double feederPowerTest = -1.0;

    /**
     * Creates a new instance of this FeederSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private FeederSubsystem() {
        m_feeder.configFactoryDefault();
        m_feeder.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Set the power of the feeder
     * @param power (double) -1.0 to 1.0
     */
    public void setPower(double power) {
        m_feeder.set(ControlMode.PercentOutput, power);
    }

    public void updateFeederPower() {
        feederPowerTest = Constants.updateConstant("Feeder Power", feederPowerTest,
                -1.0, 1.0);
    }

}

