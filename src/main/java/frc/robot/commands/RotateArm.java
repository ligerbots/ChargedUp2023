package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.armsimulation.ArmSubsystem;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.DriveTrain;
import java.util.function.DoubleSupplier;

public class RotateArm extends CommandBase {
    private final RotateArm m_rotatearm;

    private final DoubleSupplier m_rotationYSupplier;

    public RotateArm(DoubleSupplier rotationYSupplier)
            this.m_rotationYSupplier = rotationYSupplier;
 }

    @Override
    public void execute() {
        SingleJointedArmSim.m_armSim
        new SingleJointedArmSim(

                m_rotationYSupplier.getAsDouble()));

     

}


