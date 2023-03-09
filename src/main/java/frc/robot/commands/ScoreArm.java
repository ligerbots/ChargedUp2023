// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Constants.Position;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Reacher;
import frc.robot.subsystems.Shoulder;

public class ScoreArm extends CommandBase {

    private static final double LOW_GIRD_ANGLE = Math.toRadians(-45.0);
    private static final double LOW_GIRD_LENGTH = Units.inchesToMeters(8.0);

    private static final double MIDDLE_GRID_CUBE_ANGLE = Math.toRadians(-7.0);
    private static final double MIDDLE_GRID_CUBE_LENGTH = Units.inchesToMeters(12.0);
    private static final double MIDDLE_GRID_CONE_ANGLE = Math.toRadians(8.0);
    private static final double MIDDLE_GRID_CONE_LENGTH = Units.inchesToMeters(14.0);

    private static final double HIGH_GRID_CUBE_ANGLE = Math.toRadians(10.0);
    private static final double HIGH_GRID_CUBE_LENGTH = Units.inchesToMeters(31.0);
    private static final double HIGH_GRID_CONE_ANGLE = Math.toRadians(18.0);
    private static final double HIGH_GRID_CONE_LENGTH = Units.inchesToMeters(33.0);

    private static final double FLOOR_PICK_UP_CONE_ANGLE = Math.toRadians(-55.0);
    private static final double FLOOR_PICK_UP_CONE_LENGTH = Units.inchesToMeters(9.0);
    // private static final double FLOOR_PICK_UP_CUBE_ANGLE = Math.toRadians(-56.0);
    // private static final double FLOOR_PICK_UP_CUBE_LENGTH = Units.inchesToMeters(6.0);

    private static final double SUBSTATION_ANGLE = Math.toRadians(12.0);
    private static final double SUBSTATION_LENGTH = Units.inchesToMeters(1.0);

    private static final double STOW_ANGLE = Math.toRadians(-65.0);
    private static final double STOW_LENGTH = Units.inchesToMeters(1.0);

    private static final HashMap<Position, Pair<Double, Double>> SCORE_POSITIONS = new HashMap<Position, Pair<Double, Double>>(){
        {
            // scoring arm length and angle (angle, length)
            put(Position.LEFT_TOP, new Pair<>(HIGH_GRID_CONE_ANGLE, HIGH_GRID_CONE_LENGTH));
            put(Position.CENTER_TOP, new Pair<>(HIGH_GRID_CUBE_ANGLE, HIGH_GRID_CUBE_LENGTH));
            put(Position.RIGHT_TOP, new Pair<>(HIGH_GRID_CONE_ANGLE, HIGH_GRID_CONE_LENGTH));

            put(Position.LEFT_MIDDLE, new Pair<>(MIDDLE_GRID_CONE_ANGLE, MIDDLE_GRID_CONE_LENGTH));
            put(Position.CENTER_MIDDLE, new Pair<>(MIDDLE_GRID_CUBE_ANGLE, MIDDLE_GRID_CUBE_LENGTH));
            put(Position.RIGHT_MIDDLE, new Pair<>(MIDDLE_GRID_CONE_ANGLE, MIDDLE_GRID_CONE_LENGTH));

            put(Position.LEFT_BOTTOM, new Pair<>(LOW_GIRD_ANGLE, LOW_GIRD_LENGTH));
            put(Position.CENTER_BOTTOM, new Pair<>(LOW_GIRD_ANGLE, LOW_GIRD_LENGTH));
            put(Position.RIGHT_BOTTOM, new Pair<>(LOW_GIRD_ANGLE, LOW_GIRD_LENGTH));

            // substation positions
            put(Position.LEFT_SUBSTATION, new Pair<>(SUBSTATION_ANGLE, SUBSTATION_LENGTH));
            put(Position.RIGHT_SUBSTATION, new Pair<>(SUBSTATION_ANGLE, SUBSTATION_LENGTH));

            put(Position.PICK_UP, new Pair<>(FLOOR_PICK_UP_CONE_ANGLE, FLOOR_PICK_UP_CONE_LENGTH));
            put(Position.STOW_ARM, new Pair<>(STOW_ANGLE, STOW_LENGTH));            
        }
    };

    boolean m_goingDown;
    Arm m_arm;
    DriveTrain m_driveTrain;

    double m_desiredAngle;
    double m_desiredLength;
    Constants.Position m_position;

    boolean m_cancel;

    /** Creates a new ScoreArm. */
    public ScoreArm(Arm arm, DriveTrain driveTrain, Constants.Position position) {
        m_arm = arm;
        m_driveTrain = driveTrain;
        m_position = position;
        m_desiredAngle = Shoulder.limitShoulderAngle(SCORE_POSITIONS.get(position).getFirst());
        m_desiredLength = Reacher.limitReacherLength(SCORE_POSITIONS.get(position).getSecond());
        
        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        m_cancel = false;
        SmartDashboard.putString("armCommands/CommandName", m_position.toString());
        SmartDashboard.putBoolean("armCommands/isCommandFinished", false);

        // prevent from stowing in the bad zone
        if (m_position == Position.STOW_ARM) {
            double currentX = m_driveTrain.getPose().getX();
            // System.out.println("testing stow " + currentX);

            // check if the robot position is within the safe zone on either side of field, if so then end command
            if(currentX < FieldConstants.BAD_ZONE_X_BLUE || currentX > FieldConstants.BAD_ZONE_X_RED) {
                // System.out.println("***********cancelling***************");
                m_cancel = true;
                return ;
            }
        }

        m_goingDown = m_desiredAngle < m_arm.getArmAngle();

        if (m_goingDown)
            m_arm.setArmLength(m_desiredLength);
        else
            m_arm.setArmAngle(m_desiredAngle);
    }

    @Override
    public void execute() {
        if (m_cancel)
            return;

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
        System.out.println("*** ScoreArm end interrupted = " + interrupted);
        SmartDashboard.putBoolean("armCommands/isCommandFinished", true);
    }

    @Override
    public boolean isFinished() {
        if (m_cancel)
            return true;

        double curLength = m_arm.getArmLength();
        double curAngle = m_arm.getArmAngle();
        return Math.abs(curAngle - m_desiredAngle) < Shoulder.SHOULDER_ANGLE_TOLERANCE_RADIAN && 
                Math.abs(curLength - m_desiredLength) < Reacher.REACHER_OFFSET_TOLERANCE_METERS;        
    }
}
