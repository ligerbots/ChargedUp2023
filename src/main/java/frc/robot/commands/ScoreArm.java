// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Position;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Reacher;
import frc.robot.subsystems.Shoulder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreArm extends CommandBase{

    private static final Map<Position, Pair<Double, Double>> SCORE_POSITIONS = new HashMap<Position, Pair<Double, Double>>(){
        {
            // scoring arm length and angle (angle, length)
            put(Position.LEFT_TOP, new Pair<>(Constants.HIGH_GRID_CONE_ARM_ANGLE, Constants.HIGH_GRID_CONE_ARM_LENGTH));
            put(Position.CENTER_TOP, new Pair<>(Constants.HIGH_GRID_CUBE_ARM_ANGLE, Constants.HIGH_GRID_CUBE_ARM_LENGTH));
            put(Position.RIGHT_TOP, new Pair<>(Constants.HIGH_GRID_CONE_ARM_ANGLE, Constants.HIGH_GRID_CONE_ARM_LENGTH));

            put(Position.LEFT_MIDDLE, new Pair<>(Constants.MIDDLE_GRID_CONE_ARM_ANGLE, Constants.MIDDLE_GRID_CONE_ARM_LENGTH));
            put(Position.CENTER_MIDDLE, new Pair<>(Constants.MIDDLE_GRID_CUBE_ARM_ANGLE, Constants.MIDDLE_GRID_CUBE_ARM_LENGTH));
            put(Position.RIGHT_MIDDLE, new Pair<>(Constants.MIDDLE_GRID_CONE_ARM_ANGLE, Constants.MIDDLE_GRID_CONE_ARM_LENGTH));

            put(Position.LEFT_BOTTOM, new Pair<>(Constants.LOW_GIRD_ARM_ANGLE, Constants.LOW_GIRD_ARM_LENGTH));
            put(Position.CENTER_BOTTOM, new Pair<>(Constants.LOW_GIRD_ARM_ANGLE, Constants.LOW_GIRD_ARM_LENGTH));
            put(Position.RIGHT_BOTTOM, new Pair<>(Constants.LOW_GIRD_ARM_ANGLE, Constants.LOW_GIRD_ARM_LENGTH));
            // substation positions, change later
            // NOTE substation left/right is flipped because we are going with the Driver's perspective
            
            //TODO: get constants for the substations
            put(Position.LEFT_SUBSTATION, new Pair<>(Constants.SUBSTATION_ANGLE, Constants.SUBSTATION_LENGTH));
            put(Position.RIGHT_SUBSTATION, new Pair<>(Constants.SUBSTATION_ANGLE, Constants.SUBSTATION_LENGTH));

            put(Position.PICK_UP, new Pair<>(Constants.FLOOR_PICK_UP_CONE_ANGLE, Constants.FLOOR_PICK_UP_CONE_LENGTH));
            put(Position.STOW_ARM, new Pair<>(Constants.STOW_ARM_ANGLE, Constants.STOW_ARM_LENGTH));            
        }
    };

    boolean m_goingDown;
    Arm m_arm;

    double m_desiredAngle;
    double m_desiredLength;

    Constants.Position m_position;
    /** Creates a new ScoreArm. */
    public ScoreArm(Arm arm, Constants.Position position) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        m_arm = arm;
        m_position = position;
        m_desiredAngle = Shoulder.limitShoulderAngle(SCORE_POSITIONS.get(position).getFirst());
        m_desiredLength = Reacher.limitReacherLength(SCORE_POSITIONS.get(position).getSecond());
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("armCommands/CommandName", m_position.toString());
        SmartDashboard.putBoolean("armCommands/isCommandFinished", false);
        m_goingDown = m_desiredAngle < m_arm.getArmAngle();

        if(m_goingDown)
            m_arm.setArmLength(m_desiredLength);
        else
            m_arm.setArmAngle(m_desiredAngle);
    }

    @Override
    public void execute() {
        if (m_goingDown) {
            double curLength = m_arm.getArmLength();
            if (Math.abs(curLength - m_desiredLength) < Reacher.REACHER_OFFSET_TOLERANCE_METERS) {
                m_arm.setArmAngle(m_desiredAngle);
            }
        } else {
            double curAngle = m_arm.getArmAngle();
            if (Math.abs(curAngle - m_desiredAngle) < Shoulder.SHOULDER_ANGLE_TOLERANCE_RADIAN) {
                m_arm.setArmLength(m_desiredLength);
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("armCommands/isCommandFinished", true);
    }

    @Override
    public boolean isFinished() {
        double curLength = m_arm.getArmLength();
        double curAngle = m_arm.getArmAngle();
        return Math.abs(curAngle - m_desiredAngle) < Shoulder.SHOULDER_ANGLE_TOLERANCE_RADIAN && 
                Math.abs(curLength - m_desiredLength) < Reacher.REACHER_OFFSET_TOLERANCE_METERS;        
    }
}
