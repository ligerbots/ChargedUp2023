package frc.robot.swerve;

import com.revrobotics.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

// LigerBots SteerController for Swerve

public class NeoSteerController {
    // we use this all over the place, so keep it as a constant
    private static final double TWO_PI = 2.0 * Math.PI;

    private static final int ENCODER_RESET_ITERATIONS = 500;
    private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);

    private static final double CURRENT_LIMIT = 20.0;

    // PID configuration
    private static final double PID_PROPORTIONAL = 1.0;
    private static final double PID_INTEGRAL = 0.0;
    private static final double PID_DERIVATIVE = 0.1;
    private static final boolean MOTOR_INVERTED = false;
    private static final double STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);

    private final CANSparkMax m_motor;
    private final SparkMaxPIDController m_controller;
    private final RelativeEncoder m_motorEncoder;
    private final CanCoderWrapper m_absoluteEncoder;

    private double m_referenceAngleRadians = 0;
    private double m_resetIteration = 0;

    static void checkNeoError(REVLibError error, String message) {
        if (error != REVLibError.kOk) {
            DriverStation.reportError(String.format("%s: %s", message, error.toString()), false);
            System.out.println(String.format("%s: %s", message, error.toString()));
        }
    }

    public NeoSteerController(int canId, int canCoderCanId, double angleOffsetRadians) {
        // absolute angle encoder CANcoder
        m_absoluteEncoder = new CanCoderWrapper(canCoderCanId, angleOffsetRadians);

        // the turn motor
        m_motor = new CANSparkMax(canId, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();

        // adjust the CANbus update periods and alert on any errors
        checkNeoError(m_motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100),
                "Failed to set periodic status frame 0 rate");
        checkNeoError(m_motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20),
                "Failed to set periodic status frame 1 rate");
        checkNeoError(m_motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20),
                "Failed to set periodic status frame 2 rate");

        // set turn moter to brake mode
        checkNeoError(m_motor.setIdleMode(CANSparkMax.IdleMode.kBrake), "Failed to set NEO idle mode");
        m_motor.setInverted(!MOTOR_INVERTED);

        // enable voltage compensation and current limit
        checkNeoError(m_motor.enableVoltageCompensation(Constants.MAX_VOLTAGE), "Failed to enable voltage compensation");
        checkNeoError(m_motor.setSmartCurrentLimit((int) Math.round(CURRENT_LIMIT)),
                "Failed to set NEO current limits");

        // the encoder in the motor
        m_motorEncoder = m_motor.getEncoder();

        // set the builtin encoder scaling for distance and speed
        checkNeoError(m_motorEncoder.setPositionConversionFactor(TWO_PI * STEER_REDUCTION),
                "Failed to set NEO encoder conversion factor");
        checkNeoError(m_motorEncoder.setVelocityConversionFactor(TWO_PI * STEER_REDUCTION / 60.0),
                "Failed to set NEO encoder conversion factor");

        // set the built in encoder to match the CANcoder
        checkNeoError(m_motorEncoder.setPosition(m_absoluteEncoder.getAbsoluteAngleRadians()),
                "Failed to set NEO encoder position");

        // PID controller to maintain the turn angle
        m_controller = m_motor.getPIDController();
        checkNeoError(m_controller.setP(PID_PROPORTIONAL), "Failed to set NEO PID proportional constant");
        checkNeoError(m_controller.setI(PID_INTEGRAL), "Failed to set NEO PID integral constant");
        checkNeoError(m_controller.setD(PID_DERIVATIVE), "Failed to set NEO PID derivative constant");
        checkNeoError(m_controller.setFeedbackDevice(m_motorEncoder), "Failed to set NEO PID feedback device");
    }

    // get the angle setpoint, in radians
    public double getReferenceAngle() {
        return m_referenceAngleRadians;
    }

    // synchronize the angle encoder offsets
    public void syncAngleEncoders(boolean dontCheckTimer) {
        // Reset the NEO's encoder periodically when the module is not rotating.
        // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't
        // fully set up, and we don't
        // end up getting a good reading. If we reset periodically this won't matter
        // anymore.

        if (dontCheckTimer) {
            // System.out.println("** Synchronizing swerve angle encoders");
            m_motorEncoder.setPosition(m_absoluteEncoder.getAbsoluteAngleRadians());
            m_resetIteration = 0;
            return;
        }

        if (m_motorEncoder.getVelocity() < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
            if (++m_resetIteration >= ENCODER_RESET_ITERATIONS) {
                // System.out.println("** Synchronizing swerve angle encoders");
                m_motorEncoder.setPosition(m_absoluteEncoder.getAbsoluteAngleRadians());
                m_resetIteration = 0;
            }
        } else {
            m_resetIteration = 0;
        }
    }

    // set the angle we want for the wheel (radians)
    public void setReferenceAngle(double referenceAngleRadians) {
        double currentAngleRadians = m_motorEncoder.getPosition();

        // force into 0 -> 2*PI
        double currentAngleRadiansMod = currentAngleRadians % TWO_PI;
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += TWO_PI;
        }

        // The reference angle has the range [0, 2pi) but the Neo's encoder can go above that
        double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
        if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= TWO_PI;
        } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += TWO_PI;
        }

        m_referenceAngleRadians = referenceAngleRadians;

        m_controller.setReference(adjustedReferenceAngleRadians, CANSparkMax.ControlType.kPosition);
    }

    // get the current module angle in radians
    public Rotation2d getStateAngle() {
        double motorAngleRadians = m_motorEncoder.getPosition();
        motorAngleRadians %= TWO_PI;
        if (motorAngleRadians < 0.0) {
            motorAngleRadians += TWO_PI;
        }

        return Rotation2d.fromRadians(motorAngleRadians);
    }

    public void updateSmartDashboard(String sdPrefix) {
        SmartDashboard.putNumber(sdPrefix + "/angle", getStateAngle().getDegrees());

        // Compute the calibration angle for this module
        // Only use the value if the wheels are physically aligned forward, with bevel gear on the left
        // NOTE: we want a negative angle, -2PI -> 0
        double calibAngle = m_absoluteEncoder.getOffsetAngleRadians() - m_absoluteEncoder.getAbsoluteAngleRadians();
        if (calibAngle > 0.0) calibAngle -= TWO_PI;
        if (calibAngle < -TWO_PI) calibAngle += TWO_PI;
        SmartDashboard.putNumber(sdPrefix + "/calibrationAngle", Math.toDegrees(calibAngle));

        double offset = getStateAngle().getRadians() - m_absoluteEncoder.getAbsoluteAngleRadians();
        if (offset > Math.PI) offset -= TWO_PI;
        if (offset < -Math.PI) offset += TWO_PI;
        SmartDashboard.putNumber(sdPrefix + "/cancoder_offset", Math.toDegrees(offset));
    }
}
