package frc.robot.swerve;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

import edu.wpi.first.wpilibj.DriverStation;

// A wrapper around the CANCoder absolute angle sensor

public class CanCoderWrapper {
    private static final int PERIOD_MILLISECONDS = 100;
    private static final boolean ROTATION_CLOCKWISE = false;

    private final CANCoder m_encoder;

    // remember the offsetAngle to simplify recalibration of the offset
    private final double m_offsetAngleRadians;

    public static void checkCtreError(ErrorCode errorCode, String message) {
        if (errorCode != ErrorCode.OK) {
            DriverStation.reportError(String.format("%s: %s", message, errorCode.toString()), false);
            // System.out.println("** ERROR in config of CANCoder: " + errorCode.toString());
        }
    }

    public CanCoderWrapper(int canId, double offsetRadians) {
        m_encoder = new CANCoder(canId);
        m_offsetAngleRadians = offsetRadians;

        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.magnetOffsetDegrees = Math.toDegrees(offsetRadians);
        config.sensorDirection = ROTATION_CLOCKWISE;

        // set the update period and report any errors
        checkCtreError(m_encoder.configAllSettings(config, 250), "Failed to configure CANCoder");

        checkCtreError(m_encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, PERIOD_MILLISECONDS, 250),
                "Failed to configure CANCoder update rate");
    };

    public double getOffsetAngleRadians() {
        return m_offsetAngleRadians;
    }

    // get the absolute angle, in radians
    public double getAbsoluteAngleRadians() {
        double angle = Math.toRadians(m_encoder.getAbsolutePosition());
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }
}
