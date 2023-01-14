package frc.robot.swerve;

import com.revrobotics.*;

import edu.wpi.first.wpilibj.DriverStation;

// public static final ModuleConfiguration MK4I_L2 = new ModuleConfiguration(
//     0.10033,
//     (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
//     true,
//     (14.0 / 50.0) * (10.0 / 60.0),
//     false
// );


// private static SteerControllerFactory<?, NeoSteerConfiguration<CanCoderAbsoluteConfiguration>> getNeoSteerFactory(Mk4ModuleConfiguration configuration) {
//     return new NeoSteerControllerFactoryBuilder()
//             .withVoltageCompensation(configuration.getNOMINAL_VOLTAGE())
//             .withPidConstants(1.0, 0.0, 0.1)
//             .withCurrentLimit(configuration.getSteerCurrentLimit())
//             .build(new CanCoderFactoryBuilder()
//                     .withReadingUpdatePeriod(100)
//                     .build());
// }


public class NeoSteerController {
    private static final int ENCODER_RESET_ITERATIONS = 500;
    private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);

    private double NOMINAL_VOLTAGE = 12.0;
    private final double CURRENT_LIMIT = 20.0;

    // PID configuration
    private double PID_PROPORTIONAL = 1.0;
    private double PID_INTEGRAL = 0.0;
    private double PID_DERIVATIVE = 0.1;
    private final boolean MOTOR_INVERTED = false; 

    private final CANSparkMax m_motor;
    private final SparkMaxPIDController m_controller;
    private final RelativeEncoder m_motorEncoder;
    private final CanCoder m_absoluteEncoder;

    private double m_referenceAngleRadians = 0;
    private final double STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);
    private double m_resetIteration = 0;

    public static void checkNeoError(REVLibError error, String message) {
        if (error != REVLibError.kOk) {
            DriverStation.reportError(String.format("%s: %s", message, error.toString()), false);
        }
    }

    public NeoSteerController(int canId, int canCoderCanId, double angleOffset) {
        m_absoluteEncoder = new CanCoder(canCoderCanId, angleOffset);

        m_motor = new CANSparkMax(canId, CANSparkMaxLowLevel.MotorType.kBrushless);
        checkNeoError(m_motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100),
                "Failed to set periodic status frame 0 rate");
        checkNeoError(m_motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20),
                "Failed to set periodic status frame 1 rate");
        checkNeoError(m_motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20),
                "Failed to set periodic status frame 2 rate");
        checkNeoError(m_motor.setIdleMode(CANSparkMax.IdleMode.kBrake), "Failed to set NEO idle mode");
        m_motor.setInverted(!MOTOR_INVERTED);
        checkNeoError(m_motor.enableVoltageCompensation(NOMINAL_VOLTAGE), "Failed to enable voltage compensation");
        checkNeoError(m_motor.setSmartCurrentLimit((int) Math.round(CURRENT_LIMIT)),
                "Failed to set NEO current limits");

        m_motorEncoder = m_motor.getEncoder();

        checkNeoError(
                m_motorEncoder.setPositionConversionFactor(2.0 * Math.PI * STEER_REDUCTION), "Failed to set NEO encoder conversion factor");
        checkNeoError(
                m_motorEncoder
                        .setVelocityConversionFactor(2.0 * Math.PI * STEER_REDUCTION / 60.0),
                "Failed to set NEO encoder conversion factor");
        checkNeoError(m_motorEncoder.setPosition(m_absoluteEncoder.getAbsoluteAngle()),
                "Failed to set NEO encoder position");

        m_controller = m_motor.getPIDController();
            checkNeoError(m_controller.setP(PID_PROPORTIONAL), "Failed to set NEO PID proportional constant");
            checkNeoError(m_controller.setI(PID_INTEGRAL), "Failed to set NEO PID integral constant");
            checkNeoError(m_controller.setD(PID_DERIVATIVE), "Failed to set NEO PID derivative constant");
        checkNeoError(m_controller.setFeedbackDevice(m_motorEncoder), "Failed to set NEO PID feedback device");

    }

    public double getReferenceAngle() {
        return m_referenceAngleRadians;
    }

    public void setReferenceAngle(double referenceAngleRadians) {
        double currentAngleRadians = m_motorEncoder.getPosition();

        // Reset the NEO's encoder periodically when the module is not rotating.
        // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't
        // fully set up, and we don't
        // end up getting a good reading. If we reset periodically this won't matter
        // anymore.
        if (m_motorEncoder.getVelocity() < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
            if (++m_resetIteration >= ENCODER_RESET_ITERATIONS) {
                m_resetIteration = 0;
                double absoluteAngle = m_absoluteEncoder.getAbsoluteAngle();
                m_motorEncoder.setPosition(absoluteAngle);
                currentAngleRadians = absoluteAngle;
            }
        } else {
            m_resetIteration = 0;
        }

        double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += 2.0 * Math.PI;
        }

        // The reference angle has the range [0, 2pi) but the Neo's encoder can go above
        // that
        double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
        if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI;
        }

       m_referenceAngleRadians = referenceAngleRadians;

        m_controller.setReference(adjustedReferenceAngleRadians, CANSparkMax.ControlType.kPosition);
    }

    public double getStateAngle() {
        double motorAngleRadians = m_motorEncoder.getPosition();
        motorAngleRadians %= 2.0 * Math.PI;
        if (motorAngleRadians < 0.0) {
            motorAngleRadians += 2.0 * Math.PI;
        }

        return motorAngleRadians;
    }
}
