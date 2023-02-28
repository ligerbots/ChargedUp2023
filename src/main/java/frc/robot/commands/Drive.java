package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

import java.util.function.DoubleSupplier;

public class Drive extends CommandBase {
    private final DriveTrain m_driveTrain;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    public Drive(DriveTrain driveTrain,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier) {
        this.m_driveTrain = driveTrain;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        
        addRequirements(m_driveTrain);
    }

    @Override
    public void execute() {
        m_driveTrain.joystickDrive(m_translationXSupplier.getAsDouble(),  m_translationYSupplier.getAsDouble(), m_rotationSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_driveTrain.stop();
    }
}
