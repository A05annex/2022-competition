package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.lang.invoke.ConstantBootstraps;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Hashtable;

public class TestMotorsSubsystem extends SubsystemBase {
    /**
     * The Singleton instance of this TestMotorsSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private static TestMotorsSubsystem INSTANCE;

    /**
     * Returns the Singleton instance of this TestMotorsSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code TestMotorsSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static TestMotorsSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new TestMotorsSubsystem();
        }
        return INSTANCE;
    }

    private final HashMap<String, TalonSRX> motorList = new HashMap<>();

    /**
     * Creates a new instance of this TestMotorsSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private TestMotorsSubsystem() {
        // add and instantiates all motors from MOTORS_TO_TEST to motorList

        final HashMap<String, Integer> motorsToTest = Constants.getMotorsToTest();

        for (String key : motorsToTest.keySet()) {
            int port = motorsToTest.get(key);
            TalonSRX motor = new TalonSRX(port);
            motor.configFactoryDefault();
            motor.setNeutralMode(NeutralMode.Coast);
            motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
            motorList.put(key, motor);
        }
    }

    public HashMap<String, TalonSRX> getMotorList() {
        return motorList;
    }
}

