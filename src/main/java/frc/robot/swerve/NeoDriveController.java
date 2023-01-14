package frc.robot.swerve;

public interface NeoDriveController {
    void setReferenceVoltage(double voltage);

    double getStateVelocity();
}
