package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.ctre.phoenix.sensors.CANCoder;

import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.platform.runner.JUnitPlatform;
import org.junit.runner.RunWith;
import org.mockito.AdditionalMatchers;
import org.mockito.ArgumentMatchers;

import static org.junit.jupiter.api.Assertions.*;

import static org.mockito.Mockito.*;

/**
 *
 */
@RunWith(JUnitPlatform.class)
public class TestMk4NeoModule {

    /**
     * This is a class that creates an initialized drive module using mocked motor controllers, motor encoders,
     * motor PID and analog encoder.
     */
    private class InitializedMk4NeoModule {
        // basic code representations for physical hardware
        final CANSparkMax driveMotor = mock(CANSparkMax.class);
        final CANSparkMax spinMotor = mock(CANSparkMax.class);
        final CANCoder analogEncoder = mock(CANCoder.class);
        // derived representations of components embedded in the physical hardware
        final RelativeEncoder driveEncoder = mock(RelativeEncoder.class);
        final SparkMaxPIDController drivePID = mock(SparkMaxPIDController.class);
        final RelativeEncoder spinEncoder = mock(RelativeEncoder.class);
        final SparkMaxPIDController spinPID = mock(SparkMaxPIDController.class);
        final Mk4NeoModule driveModule;

        /**
         * Instantiate, initialize, and verify initialization called everything as expected. Reset all of the mock
         * objects that were touched in initialization before exiting this instantiation.
         */
        public InitializedMk4NeoModule() {
            when(analogEncoder.getAbsolutePosition()).thenReturn(Math.PI/2.0);
            driveModule = new Mk4NeoModule(driveMotor, driveEncoder, drivePID,
                    spinMotor, spinEncoder, spinPID,
                    analogEncoder, -(Math.PI/2.0));
            assertEquals(Math.PI/2.0, driveModule.getCalibrationPosition());
            // In this test example, the wheel is facing directly backwards, so the position should be set to
            // half a direction revolution.
            verifyPid(drivePID, Mk4NeoModule.DRIVE_kFF, Mk4NeoModule.DRIVE_kP,
                    Mk4NeoModule.DRIVE_kI, Mk4NeoModule.DRIVE_IZONE);
            verifyPid(spinPID, 0.0, Mk4NeoModule.SPIN_kP, Mk4NeoModule.SPIN_kI, 0.0);
            verify(spinEncoder, times(1)).setPosition(Math.PI * Mk4NeoModule.RADIANS_TO_SPIN_ENCODER);
            verify(spinPID, times(1)).setReference(0.0, CANSparkMax.ControlType.kPosition);
            reset(spinEncoder, drivePID, spinPID);
        }
    }

    /**
     *
     * @param pid
     * @param kFF
     * @param kP
     * @param kI
     * @param kIZone
     */
    private void verifyPid(SparkMaxPIDController pid, double kFF, double kP, double kI, double kIZone) {
        verify(pid, times(1)).setFF(kFF);
        verify(pid, times(1)).setP(kP);
        verify(pid, times(1)).setI(kI);
        verify(pid, times(1)).setD(0.0);
        verify(pid, times(1)).setIZone(kIZone);
        verify(pid, times(1)).setOutputRange(-1.0, 1.0);
    }

    /**
     * The deal here is that we are setting a direction and speed for the module, but the actual direction
     * may be reversed 180, or the module may have gone over the 180 degree spin boundary, so the actual
     * may not be what was really set, but something that gives the same result.
     *
     * @param dm
     * @param radians
     * @param speed
     * @param actualRadians
     * @param actualSpeed
     */
    private void verifyDirectionAndSpeed(InitializedMk4NeoModule dm, double radians, double speed,
                                         double actualRadians, double actualSpeed) {
        dm.driveModule.setDirectionAndSpeed(new AngleD(AngleUnit.RADIANS, radians), speed);
        verify(dm.spinPID, times(1)).setReference(
                AdditionalMatchers.eq(actualRadians * Mk4NeoModule.RADIANS_TO_SPIN_ENCODER,.00001),
                ArgumentMatchers.eq(CANSparkMax.ControlType.kPosition));
        verify(dm.drivePID, times(1)).setReference(actualSpeed * Mk4NeoModule.MAX_DRIVE_RPM,
                CANSparkMax.ControlType.kVelocity);
        // make encoder readings slightly different than what was actually set so that when we get these
        // things we know we are really getting them from the encoder.
        double spinEncPosition = (actualRadians * Mk4NeoModule.RADIANS_TO_SPIN_ENCODER) + .001;
        when(dm.spinEncoder.getPosition()).thenReturn(spinEncPosition);
        double driveEncPosition = Math.random() * 1000.0; // we have no idea what happens with this
        when(dm.driveEncoder.getPosition()).thenReturn(driveEncPosition);
        double driveEncVelocity = (actualSpeed * Mk4NeoModule.MAX_DRIVE_RPM) + 3.0;
        when(dm.driveEncoder.getVelocity()).thenReturn(driveEncVelocity);

        assertEquals(radians, dm.driveModule.getLastDirection().getRadians());
        assertEquals(speed, dm.driveModule.getLastNormalizedSpeed());
        assertEquals(speed * Mk4NeoModule.MAX_DRIVE_RPM, dm.driveModule.getLastSpeed());

        assertEquals(spinEncPosition, dm.driveModule.getDirectionPosition());
        assertEquals(driveEncPosition, dm.driveModule.getDriveEncoderPosition());
        assertEquals(driveEncVelocity, dm.driveModule.getDriveEncoderVelocity());
    }

    /**
     * This just does the instantiation and verifies the initial calibration. If there is a problem here
     * then all the other tests will probably fail.
     */
    @Test
    @DisplayName("Test Calibration")
    void test_calibration() {
        new InitializedMk4NeoModule();
    }

    /**
     * Test setting the spin PID K values
     */
    @Test
    @DisplayName("Test setSpinPID")
    void test_setSpinPID() {
        // Should spin positively
        InitializedMk4NeoModule dm = new InitializedMk4NeoModule();
        dm.driveModule.setSpinPID();
        verify(dm.spinPID, times(1)).setP(Mk4NeoModule.SPIN_kP);
        verify(dm.spinPID, times(1)).setI(Mk4NeoModule.SPIN_kI);
    }

    /**
     * Test setting the drive PID K values
     */
    @Test
    @DisplayName("Test setDrivePID")
    void test_setDrivePID() {
        // Should spin positively
        InitializedMk4NeoModule dm = new InitializedMk4NeoModule();
        dm.driveModule.setDrivePID();
        verify(dm.drivePID, times(1)).setFF(Mk4NeoModule.DRIVE_kFF);
        verify(dm.drivePID, times(1)).setP(Mk4NeoModule.DRIVE_kP);
        verify(dm.drivePID, times(1)).setI(Mk4NeoModule.DRIVE_kI);
    }

    /**
     * Test a basic move - 10 degrees clockwise, full speed forward.
     */
    @Test
    @DisplayName("Test setRadiansAndSpeed(Math.toRadians(10.0),1.0)")
    void test_set_10_1() {
        // Should spin positively
        InitializedMk4NeoModule dm = new InitializedMk4NeoModule();
        verifyDirectionAndSpeed(dm,Math.toRadians(10.0), 1.0,Math.toRadians(10.0), 1.0);
    }

//    /**
//     * Test a basic move specified in degrees - 10 degrees clockwise, full speed forward.
//     */
//    @Test
//    @DisplayName("Test setDegreesAndSpeed(10.0,1.0)")
//    void test_set_degrees_10_1() {
//        // Should spin positively
//        InitializedMk4NeoModule dm = new InitializedMk4NeoModule();
//        dm.driveModule.setDegreesAndSpeed(10.0, 1.0);
//        verify(dm.spinPID, times(1)).setReference(Math.toRadians(10.0) * Constants.RADIANS_TO_SPIN_ENCODER,
//                CANSparkMax.ControlType.kPosition);
//        verify(dm.drivePID, times(1)).setReference(1.0 * Constants.MAX_DRIVE_VELOCITY,
//                CANSparkMax.ControlType.kVelocity);
//    }

    /**
     * Test a close to, but less than, 90 degree clockwise, half speed. Spin should be clockwise and spped forward
     */
    @Test
    @DisplayName("Test setRadiansAndSpeed(Math.toRadians(80.0),0.5)")
    void test_set_80_1() {
        // Should spin positively
        InitializedMk4NeoModule dm = new InitializedMk4NeoModule();
        verifyDirectionAndSpeed(dm,Math.toRadians(80.0), 0.5,Math.toRadians(80.0), 0.5);
    }

    /**
     * Test a slightly greater than 90 degree clockwise, full speed. The closest rotation is to orient
     * the back of the wheel and go backwards - make sure this happens.
     */
    @Test
    @DisplayName("Test setRadiansAndSpeed(Math.toRadians(100.0),0.25)")
    void test_set_100_1() {
        // Should spin negatively and go backwards
        InitializedMk4NeoModule dm = new InitializedMk4NeoModule();
        verifyDirectionAndSpeed(dm,Math.toRadians(100.0), 0.25,Math.toRadians(-80.0), -0.25);
    }

    /**
     * Test a basic move - 10 degrees counter-clockwise, full speed.
     */
    @Test
    @DisplayName("Test setRadiansAndSpeed(Math.toRadians(-10.0),1.0)")
    void test_set_neg_10_1() {
        // Should spin negatively
        InitializedMk4NeoModule dm = new InitializedMk4NeoModule();
        verifyDirectionAndSpeed(dm,Math.toRadians(-10.0), 1.0,Math.toRadians(-10.0), 1.0);
    }

    @Test
    @DisplayName("Test setRadiansAndSpeed(Math.toRadians(-80.0),1.0)")
    void test_set_neg_80_1() {
        // Should spin negatively
        InitializedMk4NeoModule dm = new InitializedMk4NeoModule();
        verifyDirectionAndSpeed(dm,Math.toRadians(-80.0), 0.5,Math.toRadians(-80.0), 0.5);
    }

    @Test
    @DisplayName("Test setRadiansAndSpeed(Math.toRadians(-100.0),1.0)")
    void test_set_neg_100_1() {
        // Should spin positively and go backwards
        InitializedMk4NeoModule dm = new InitializedMk4NeoModule();
        verifyDirectionAndSpeed(dm,Math.toRadians(-100.0), 0.25,Math.toRadians(80.0), -0.25);
    }

    @Test
    @DisplayName("Test 180 boundary clockwise")
    void test_180_boundary_clockwise() {
        // Should spin positively and keep spinning positively at the 180 boundary - kind of a pain because we
        // need 2 less than 90 degree steps to get the front of the wheel there.
        InitializedMk4NeoModule dm = new InitializedMk4NeoModule();
        dm.driveModule.setDirectionAndSpeed(new AngleD(AngleUnit.DEGREES, 85.0), 1.0);
        dm.driveModule.setDirectionAndSpeed(new AngleD(AngleUnit.DEGREES, 170.0), 1.0);
        // This is the 180 boundary cross
        reset(dm.spinPID,dm.drivePID);
        verifyDirectionAndSpeed(dm,Math.toRadians(-170.0), 0.35,Math.toRadians(190.0), 0.35);
    }

    @Test
    @DisplayName("Test 180 boundary counter clockwise")
    void test_180_boundary_counter_clockwise() {
        // Should spin negatively and keep spinning negatively at the 180 boundary - kind of a pain because we
        // need 2 less than -90 steps to get the front of the wheel there.
        InitializedMk4NeoModule dm = new InitializedMk4NeoModule();
        dm.driveModule.setDirectionAndSpeed(new AngleD(AngleUnit.DEGREES, -85.0), 1.0);
        dm.driveModule.setDirectionAndSpeed(new AngleD(AngleUnit.DEGREES, -170.0), 1.0);
        // This is the 180 boundary cross
        reset(dm.spinPID,dm.drivePID);
        verifyDirectionAndSpeed(dm,Math.toRadians(170.0), 0.35,Math.toRadians(-190.0), 0.35);
    }

    /**
     * The forwards-backwards logic finds the least spin of the wheel (>= 90.0) and the appropriate forward/backward
     * multiplier. A concern it the programming has been making sure the forward-backward states are handled
     * correctly. Specifically, if I have 2 forward commands it a row or 2 backwards commands in a row, does the
     * module correctly remember what it is doing and not do a 180 degree spin.
     */
    @Test
    @DisplayName("Test backwards forwards")
    void test_forward_backward() {
        // Go forward 2 steps
        InitializedMk4NeoModule dm = new InitializedMk4NeoModule();
        dm.driveModule.setDirectionAndSpeed(new AngleD(AngleUnit.RADIANS, 0.0), 1.0);
        dm.driveModule.setDirectionAndSpeed(new AngleD(AngleUnit.RADIANS,0.0), 1.0);
        verify(dm.spinPID, times(2)).setReference( 0.0, CANSparkMax.ControlType.kPosition);
        verify(dm.drivePID, times(2)).setReference(1.0 * Mk4NeoModule.MAX_DRIVE_RPM,
                CANSparkMax.ControlType.kVelocity);
        // Go backwards (180 degrees) 2 steps - should be no spin and negative speed
        reset(dm.spinPID,dm.drivePID);
        dm.driveModule.setDirectionAndSpeed(new AngleD(AngleUnit.RADIANS,Math.PI), 1.0);
        dm.driveModule.setDirectionAndSpeed(new AngleD(AngleUnit.RADIANS,Math.PI), 1.0);
        verify(dm.spinPID, times(2)).setReference( 0.0, CANSparkMax.ControlType.kPosition);
        verify(dm.drivePID, times(2)).setReference(-1.0 * Mk4NeoModule.MAX_DRIVE_RPM,
                CANSparkMax.ControlType.kVelocity);
        // Go forwards again steps - should be no spin and positive speed
        reset(dm.spinPID,dm.drivePID);
        dm.driveModule.setDirectionAndSpeed(new AngleD(AngleUnit.RADIANS,0.0), 1.0);
        dm.driveModule.setDirectionAndSpeed(new AngleD(AngleUnit.RADIANS,0.0), 1.0);
        verify(dm.spinPID, times(2)).setReference( 0.0, CANSparkMax.ControlType.kPosition);
        verify(dm.drivePID, times(2)).setReference(1.0 * Mk4NeoModule.MAX_DRIVE_RPM,
                CANSparkMax.ControlType.kVelocity);
    }


    private void verifyDirectionAndDistance(InitializedMk4NeoModule dm, double radians, double deltaTics,
                                            double actualRadians) {
        double driveEncStartPosition = Math.random() * 1000.0; //generate an arbitrary start position
        when(dm.driveEncoder.getPosition()).thenReturn(driveEncStartPosition);
        dm.driveModule.setDirectionAndDistance(new AngleD(AngleUnit.RADIANS, radians), deltaTics);
        // Test the angle set
        verify(dm.spinPID, times(1)).setReference(
                AdditionalMatchers.eq(actualRadians * Mk4NeoModule.RADIANS_TO_SPIN_ENCODER,.00001),
                ArgumentMatchers.eq(CANSparkMax.ControlType.kPosition));
        // test the last call to setReference is for the correct target position, and is asking for position
        // (not speed).
        verify(dm.drivePID).setReference(driveEncStartPosition + deltaTics, CANSparkMax.ControlType.kPosition);
        // make encoder readings slightly different than what was actually set so that when we get these
        // things we know we are really getting them from the encoder.
        double spinEncPosition = (actualRadians * Mk4NeoModule.RADIANS_TO_SPIN_ENCODER) + .001;
        when(dm.spinEncoder.getPosition()).thenReturn(spinEncPosition);
        when(dm.driveEncoder.getPosition()).thenReturn(driveEncStartPosition + deltaTics);

        assertEquals(spinEncPosition, dm.driveModule.getDirectionPosition());
        assertEquals(driveEncStartPosition + deltaTics, dm.driveModule.getDriveEncoderPosition());//        assertEquals(driveEncVelocity, dm.driveModule.getDriveEncoderVelocity());
    }
    /**
     * Test a basic targeting spin.
     */
    @Test
    @DisplayName("Test setDirectionAndDistance(Math.toRadians(10.0),10.0)")
    void test_set_direction_distance_10_20() {
        // Should spin positively
        InitializedMk4NeoModule dm = new InitializedMk4NeoModule();
        verifyDirectionAndDistance(dm,Math.toRadians(10.0), 10.0,Math.toRadians(10.0));
    }
}
