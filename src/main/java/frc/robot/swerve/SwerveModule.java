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
        double currAngle = getSteerAngle().getRadians();
        double difference = NeoSteerController.wrapAngle(steerAngle - currAngle);

        // If the difference is greater than 90 deg or less than 270, the drive can
        // be inverted so the total movement of the module is less than 90 deg
        if (difference > 0.5 * Math.PI && difference < 1.5 * Math.PI)
        {
            steerAngle += Math.PI;
            driveVoltage *= -1.0;
        }

        // Put the target angle into the range [0, 2pi)
        steerAngle = NeoSteerController.wrapAngle(steerAngle);

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
