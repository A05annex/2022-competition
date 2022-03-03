package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
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

    // PID constants
    private double LIFT_kP = 0.7;
    private double LIFT_kI = 0.0008;
    private double LIFT_iZone = 3000;

    // set encoder position for test
    private double m_left_lift_enc_set = 0.0;
    private double m_right_lift_enc_set = 0.0;
    private final double m_inc = 10000.0; // amount to increment

    // amount to subtract from encoder values to see if the bar is there yet
    public static final double LIFT_TOLERANCE = 1000.0;

    // power for LiftPowerCommand
    public static final double LIFT_POWER = 1.0;

    // hanger position constants
    public static final class HangerPositions {
        public static final double
            RIGHT_UP = 400000.0, // all the way up
            LEFT_UP = 600000.0, // all the way up
            RIGHT_DOWN = 260000.0, // right angle, untested
            LEFT_DOWN = 320000, // right angle, untested
            RIGHT_AUTO_UP = 10000.0; // drop collector, untested
    }

    /**
     * Creates a new instance of this HangerSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private LiftSubsystem() {
        m_lift_left.configFactoryDefault();
        m_lift_left.setNeutralMode(NeutralMode.Coast);
        m_lift_left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        m_lift_left.setSensorPhase(true);
        m_lift_left.setSelectedSensorPosition(0.0);
        m_lift_right.configFactoryDefault();
        m_lift_right.setNeutralMode(NeutralMode.Coast);
        m_lift_right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        m_lift_right.setSensorPhase(true);
        m_lift_right.setSelectedSensorPosition(0.0);
        setLiftPID();
    }

    /**
     * Use PID to drive the left hanger.
     * @param position (double) Encoder value to set the hanger at.
     */
    public void setLeftPosition(double position) {
        m_lift_left.set(ControlMode.Position, position);
    }

    /**
     * @return The position of the left lift encoder in tics.
     */
    public double getLeftPosition() {
        return m_lift_left.getSelectedSensorPosition();
    }

    /**
     * Use PID to drive the right hanger.
     * @param position (double) Encoder value to set the hanger at.
     */
    public void setRightPosition(double position) {
        m_lift_right.set(ControlMode.Position, position);
    }

    /**
     * @return The position of the right lift encoder in tics.
     */
    public double getRightPosition() {
        return m_lift_right.getSelectedSensorPosition();
    }

    public void setLeftPower(double power) {
        m_lift_left.set(ControlMode.PercentOutput, power);
    }

    public void setRightPower(double power) {
        m_lift_right.set(ControlMode.PercentOutput, power);
    }

    public void setLeftEncoder(double position) {
        m_lift_left.setSelectedSensorPosition(position);
    }

    public void setRightEncoder(double position) {
        m_lift_right.setSelectedSensorPosition(position);
    }

    public void zeroEncoders() {
        setLeftEncoder(0.0);
        setRightEncoder(0.0);
    }

    // set positions according to our test variables
    public void setLiftPositionsToTest() {
        setLeftPosition(m_left_lift_enc_set);
        setRightPosition(m_right_lift_enc_set);
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
//        SmartDashboard.putNumber("setter left lift enc", m_left_lift_enc_set);
//        SmartDashboard.putNumber("setter right lift enc", m_right_lift_enc_set);
    }

    // change PID and enc setters with SmartDashboard
    public void updateAllLiftConstants() {
        LIFT_kP = Constants.updateConstant("Lift kP", LIFT_kP);
        LIFT_kI = Constants.updateConstant("Lift kI", LIFT_kI);
        LIFT_iZone = Constants.updateConstant("Lift iZone", LIFT_iZone);
        setLiftPID();
        //m_left_lift_enc_set = Constants.updateConstant("setter left lift enc", m_left_lift_enc_set);
        //m_right_lift_enc_set = Constants.updateConstant("setter right lift enc", m_right_lift_enc_set);
    }

    public void setLiftPID() {
        setTalonPID(m_lift_left, LIFT_kP, LIFT_kI, LIFT_iZone);
        setTalonPID(m_lift_right, LIFT_kP, LIFT_kI, LIFT_iZone);
    }

    public void setTalonPID(TalonSRX talon, double kP, double kI, double iZone) {
        talon.config_kP(0, kP);
        talon.config_kI(0, kI);
        talon.config_IntegralZone(0, iZone);
    }

    @Override
    public void periodic() {
        // telemetry
        //printLiftData();
        updateAllLiftConstants(); // this also updates PID
    }
}

