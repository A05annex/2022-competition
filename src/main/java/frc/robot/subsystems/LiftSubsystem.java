package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LiftSubsystem extends SubsystemBase {
    /**
     * The Singleton instance of this HangerSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private static LiftSubsystem INSTANCE;

    /**
     * Returns the Singleton instance of this HangerSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code HangerSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static LiftSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new LiftSubsystem();
        }
        return INSTANCE;
    }

    // motor controllers
    private final TalonSRX m_lift_left = new TalonSRX(Constants.CAN_Devices.LIFT_LEFT);
    private final TalonSRX m_lift_right = new TalonSRX(Constants.CAN_Devices.LIFT_RIGHT);

    // PID constants TODO: read defaults from a talon and set here
    private double LIFT_kP;
    private double LIFT_kI;
    private double LIFT_kF;

    // set encoder position for test
    private double m_left_lift_enc_set = 0.0;
    private double m_right_lift_enc_set = 0.0;
    private final double m_inc = 10.0; // amount to increment

    /**
     * Creates a new instance of this HangerSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private LiftSubsystem() {
        m_lift_left.configFactoryDefault();
        m_lift_left.setNeutralMode(NeutralMode.Brake);
        m_lift_left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        m_lift_right.configFactoryDefault();
        m_lift_right.setNeutralMode(NeutralMode.Brake);
        m_lift_right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        setLiftPID();
    }

    // use PID to drive the motors
    public void setLeftLiftPosition(double position) {
        m_lift_left.setSelectedSensorPosition(position);
    }

    public void setRightLiftPosition(double position) {
        m_lift_right.setSelectedSensorPosition(position);
    }

    // set positions according to our test variables
    public void setLiftPositionsToTest() {
        setLeftLiftPosition(m_left_lift_enc_set);
        setRightLiftPosition(m_right_lift_enc_set);
    }

    // increment/decrement test variables
    public void incLeftLiftPosition() {
        m_left_lift_enc_set += m_inc;
    }

    public void decLeftLiftPosition() {
        m_left_lift_enc_set -= m_inc;
    }

    public void incRightLiftPosition() {
        m_right_lift_enc_set += m_inc;
    }

    public void decRightLiftPosition() {
        m_right_lift_enc_set -= m_inc;
    }

    public void printLiftData() {
        SmartDashboard.putNumber("left lift enc", m_lift_left.getSelectedSensorPosition());
        SmartDashboard.putNumber("right lift enc", m_lift_right.getSelectedSensorPosition());
    }

    // change PID and enc setters with SmartDashboard
    public void updateAllLiftConstants() {
        LIFT_kP = Constants.updateConstant("Lift kP", LIFT_kP);
        LIFT_kI = Constants.updateConstant("Lift kI", LIFT_kI);
        LIFT_kF = Constants.updateConstant("Lift kF", LIFT_kF);
        setLiftPID();
        m_left_lift_enc_set = Constants.updateConstant("setter left lift enc", m_left_lift_enc_set);
        m_right_lift_enc_set = Constants.updateConstant("setter right lift enc", m_right_lift_enc_set);
    }

    public void setLiftPID() {
        setTalonPID(m_lift_left, LIFT_kP, LIFT_kI, LIFT_kF);
        setTalonPID(m_lift_right, LIFT_kP, LIFT_kI, LIFT_kF);
    }

    public void setTalonPID(TalonSRX talon, double kP, double kI, double kF) {
        talon.config_kP(0, kP);
        talon.config_kI(0, kI);
        talon.config_kF(0, kF);
    }

    @Override
    public void periodic() {
        // telemetry
        printLiftData();
        updateAllLiftConstants(); // this also updates PID
    }
}

