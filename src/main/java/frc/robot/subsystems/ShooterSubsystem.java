package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    /**
     * The Singleton instance of this ShooterSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private static ShooterSubsystem INSTANCE;

    /**
     * Returns the Singleton instance of this ShooterSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code ShooterSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static ShooterSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ShooterSubsystem();
        }
        return INSTANCE;
    }

    private final TalonSRX m_frontShooter = new TalonSRX(Constants.CAN_Devices.SHOOTER_FRONT);
    private final TalonSRX m_rearShooter = new TalonSRX(Constants.CAN_Devices.SHOOTER_REAR);
    private double m_lastSetFrontSpeed;
    private double m_lastSetRearSpeed;

    // Shooter speeds for testing TODO: remove when done testing
    public double frontShooterTestSpeed = 0.5;
    public double rearShooterTestSpeed = -0.5;

    // Constants
    // Shooter max RPMs
    public static double MAX_FRONT_SHOOTER_RPM = 24000.0;
    public static double MAX_REAR_SHOOTER_RPM = 29000.0;

    //Shooter PID constants
    public static double SHOOTER_kP = 0.002;
    public static double SHOOTER_kI = 0.0;
    public static double SHOOTER_kF_FRONT = 0.042;
    public static double SHOOTER_kF_REAR = 0.0335;

    /**
     * Creates a new instance of this ShooterSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private ShooterSubsystem() {
        m_frontShooter.configFactoryDefault();
        m_frontShooter.setNeutralMode(NeutralMode.Coast);
        m_frontShooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        m_frontShooter.setSensorPhase(true);
        m_frontShooter.setInverted(true);
        m_rearShooter.configFactoryDefault();
        m_rearShooter.setNeutralMode(NeutralMode.Coast);
        m_rearShooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        m_rearShooter.setSensorPhase(true);
        m_rearShooter.setInverted(true);
        updateAllPID();
        m_lastSetFrontSpeed = 0.0;
        m_lastSetRearSpeed = 0.0;
    }

    public void updateAllPID() {
        setTalonPID(m_frontShooter, SHOOTER_kP, SHOOTER_kI, SHOOTER_kF_FRONT);
        setTalonPID(m_rearShooter, SHOOTER_kP, SHOOTER_kI, SHOOTER_kF_REAR);
    }

    public void setTalonPID(TalonSRX talon, double kP, double kI, double kF) {
        talon.config_kP(0, kP);
        talon.config_kI(0, kI);
        talon.config_kF(0, kF);
    }

    /**
     * Set the front shooter speed using velocity control.
     * @param speed (double) Between -1.0 and 1.0.
     */
    public void setFrontShooter(double speed) {
        m_lastSetFrontSpeed = MAX_FRONT_SHOOTER_RPM * speed;
        if (m_lastSetFrontSpeed == 0.0) {
            m_frontShooter.set(ControlMode.PercentOutput, m_lastSetFrontSpeed);
        } else {
            m_frontShooter.set(ControlMode.Velocity, m_lastSetFrontSpeed);
        }
    }

    /**
     * Set the front shooter power.
     * @param power (double) Between -1.0 and 1.0.
     */
    public void setFrontShooterPower(double power) {
        m_frontShooter.set(ControlMode.PercentOutput, power);
    }

    /**
     * Set the rear shooter speed using velocity control.
     * @param speed (double) Between -1.0 and 1.0.
     */
    public void setRearShooter(double speed) {
        m_lastSetRearSpeed = MAX_REAR_SHOOTER_RPM * speed;
        if (m_lastSetRearSpeed == 0.0) {
            m_rearShooter.set(ControlMode.PercentOutput, m_lastSetRearSpeed);
        } else {
            m_rearShooter.set(ControlMode.Velocity, m_lastSetRearSpeed);
        }
    }

    /**
     * Set the front shooter power.
     * @param power (double) Between -1.0 and 1.0.
     */
    public void setRearShooterPower(double power) {
        m_rearShooter.set(ControlMode.PercentOutput, power);
    }

    public double getFrontShooterSpeed() {
        return m_frontShooter.getSelectedSensorVelocity();
    }

    public double getRearShooterSpeed() {
        return m_rearShooter.getSelectedSensorVelocity();
    }

    public void updateShooterConstants() {
        frontShooterTestSpeed = Constants.updateConstant("front speed", frontShooterTestSpeed,
                -1.0, 1.0);
        rearShooterTestSpeed = Constants.updateConstant("rear speed", rearShooterTestSpeed,
                -1.0, 1.0);
    }

}

