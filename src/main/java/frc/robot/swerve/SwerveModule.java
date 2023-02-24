package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

// The LigerBots SwerveModule
// This has a SteerController and a DriveController

public class SwerveModule {
    private final String m_moduleName;
    private final NeoDriveController m_driveController;
    private final NeoSteerController m_steerController;

    public SwerveModule(String moduleName, NeoDriveController driveController, NeoSteerController steerController) {
        m_moduleName = moduleName;
        m_driveController = driveController;
        m_steerController = steerController;
    }

    public double getDriveVelocity() {
        return m_driveController.getStateVelocity();
    }

    public Rotation2d getSteerAngle() {
        return m_steerController.getStateAngle();
    }

    public void set(double driveVoltage, double steerAngle) {
        steerAngle %= (2.0 * Math.PI);
        if (steerAngle < 0.0) {
            steerAngle += 2.0 * Math.PI;
        }

        double currAngle = getSteerAngle().getRadians();
        double difference = steerAngle - currAngle;
        // Change the target angle so the difference is in the range [-pi, pi) instead of [0, 2pi)
        if (difference >= Math.PI) {
            steerAngle -= 2.0 * Math.PI;
        } else if (difference < -Math.PI) {
            steerAngle += 2.0 * Math.PI;
        }
        difference = steerAngle - currAngle; // Recalculate difference

        // If the difference is greater than 90 deg or less than -90 deg the drive can
        // be inverted so the total
        // movement of the module is less than 90 deg
        if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
            // Only need to add 180 deg here because the target angle will be put back into
            // the range [0, 2pi)
            steerAngle += Math.PI;
            driveVoltage *= -1.0;
        }

        // Put the target angle back into the range [0, 2pi)
        steerAngle %= (2.0 * Math.PI);
        if (steerAngle < 0.0) {
            steerAngle += 2.0 * Math.PI;
        }

        m_driveController.setReferenceVoltage(driveVoltage);
        m_steerController.setReferenceAngle(steerAngle);
    }

    // stops individual module
    public void stopWheel() {
        m_driveController.setReferenceVoltage(0.0);
    }

    // get the swerve module position 
    public SwerveModulePosition getSwerveModulePosition(){
        return new SwerveModulePosition(m_driveController.getWheelDistance(), m_steerController.getStateAngle() );
    }

    public double getWheelDistance(){
        return m_driveController.getWheelDistance();
    }

    public void updateSmartDashboard() {
        m_steerController.updateSmartDashboard(String.format("drivetrain/%s/steer", m_moduleName));
    }

    public void syncAngleEncoders(boolean dontCheckTimer) {
        m_steerController.syncAngleEncoders(dontCheckTimer);
    }
}
