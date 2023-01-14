package frc.robot.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DriverStation;

// public static final ModuleConfiguration MK4I_L2 = new ModuleConfiguration(
//     0.10033,
//     (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
//     true,
//     (14.0 / 50.0) * (10.0 / 60.0),
//     false
// );

public class NeoDriveController {
    private CANSparkMax m_motor;
    private RelativeEncoder m_encoder;

    private final double NOMINAL_VOLTAGE = 12.0;
    private final double CURRENT_LIMIT = 20.0;
    private final boolean MOTOR_INVERTED = true; 

    public final static double DRIVE_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public final static double WHEEL_DIAMETER = 0.10033;

    public static void checkNeoError(REVLibError error, String message) {
        if (error != REVLibError.kOk) {
            DriverStation.reportError(String.format("%s: %s", message, error.toString()), false);
        }
    }

    public NeoDriveController(int motorCanId) {
        m_motor = new CANSparkMax(motorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_motor.setInverted(MOTOR_INVERTED);

        // Setup voltage compensation
            checkNeoError(m_motor.enableVoltageCompensation(NOMINAL_VOLTAGE), "Failed to enable voltage compensation");

            checkNeoError(m_motor.setSmartCurrentLimit((int) CURRENT_LIMIT), "Failed to set current limit for NEO");

        checkNeoError(m_motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100), "Failed to set periodic status frame 0 rate");
        checkNeoError(m_motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20), "Failed to set periodic status frame 1 rate");
        checkNeoError(m_motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20), "Failed to set periodic status frame 2 rate");

        // Set neutral mode to brake
        m_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Setup encoder
        m_encoder = m_motor.getEncoder();
        double positionConversionFactor = Math.PI * WHEEL_DIAMETER * DRIVE_REDUCTION;
        m_encoder.setPositionConversionFactor(positionConversionFactor);
        m_encoder.setVelocityConversionFactor(positionConversionFactor / 60.0);
    }

    public void setReferenceVoltage(double voltage) {
        m_motor.setVoltage(voltage);
    }

    public double getStateVelocity() {
        return m_encoder.getVelocity();
    }
}
