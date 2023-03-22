// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Position;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class AutoChargeStationOneCube extends SequentialCommandGroup implements AutoCommandInterface {

    private final double BACKING_MPS = 0.5;

    AutoFollowTrajectory m_traj;

    /** Creates a new AutoChargeStationOneCube */
    public AutoChargeStationOneCube(DriveTrain driveTrain, Arm arm, Vision vision, Claw claw, JoystickButton overrideButton) {
        // Note this is a quick hack: the trajectory is loaded to just get the initial Pose and the stop point for backing up.
        //  Otherwise it is not actually used.
        m_traj = new AutoFollowTrajectory(driveTrain, "c_forward_balance");

        addCommands(
            new InstantCommand(arm::retractArm),
            new WaitCommand(0.1),

            new MoveArmAndDrive(arm, driveTrain, vision, Position.CENTER_TOP, overrideButton),
            // new PrintCommand("End of MoveArmAndDrive"),
            new InstantCommand(claw::open),
            
            // back up to stow the arm
            new AutoXPositionDrive(driveTrain, m_traj.getEndPose().getX(), BACKING_MPS),
            
            new ScoreArm(arm, driveTrain, Position.STOW_ARM, overrideButton).withTimeout(5).alongWith(new InstantCommand(claw::close)),
            new ChargeStationDrive(driveTrain),
            new ChargeStationBalance(driveTrain));
                    
        // Do NOT require any Subsystems. That is handled by the subcommands.
    }

    @Override
    public Pose2d getInitialPose() {
        return m_traj.getInitialPose();
    }
}
