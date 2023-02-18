// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

public class Reacher extends TrapezoidProfileSubsystem {

    // CAN ID for the Reacher (arm extension system)
    private static final int REACHER_CAN_ID = 7; // TODO: Set CanID

    private static final double REACHER_MAX_VEL_INCH_PER_SEC = 100.0;
    private static final double REACHER_MAX_ACC_INCH_PER_SEC_SQ = 30.0;

    private static final double REACHER_INCH_PER_REVOLUTION = 0.7;

    // Feedforward constants for the reacher
    private static final double REACHER_KS = 0.182; // TODO: This may need to be tuned
    // The following constants are computed from https://www.reca.lc/arm
    private static final double REACHER_KG = 1.19;
    private static final double REACHER_KV = 7.67;
    private static final double REACHER_KA = 0.19;

    // PID Constants for the reacher PID controller
    // Since we're using Trapeziodal control, all values will be 0 except for P
    private static final double REACHER_K_P = 1.0;
    // private static final double REACHER_K_P1 = 100;
    private static final double REACHER_K_I = 0.0;
    private static final double REACHER_K_D = 0.0;
    private static final double REACHER_K_FF = 0.0;
    private static final double REACHER_OFFSET_INCH = 0.0;


    /** Creates a new Reacher. */
    // Define the motor and encoders
    private final CANSparkMax m_motor;
    private final RelativeEncoder m_encoder;
    private final SparkMaxPIDController m_PIDController;

    private final ElevatorFeedforward m_Feedforward = new ElevatorFeedforward(REACHER_KS,
                    REACHER_KG, REACHER_KV, REACHER_KA);

    private double m_kPReacher;
    private boolean m_resetReacherPos = false;
    private boolean m_coastMode = false;
    private double m_goal = 0;

    /** Creates a new Reacher. */
    public Reacher() {
        super(new TrapezoidProfile.Constraints(REACHER_MAX_VEL_INCH_PER_SEC,
        REACHER_MAX_ACC_INCH_PER_SEC_SQ));

        m_kPReacher = REACHER_K_P;

        // Create the motor, PID Controller and encoder.
        m_motor = new CANSparkMax(REACHER_CAN_ID, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();

        m_PIDController = m_motor.getPIDController();
        m_PIDController.setP(REACHER_K_P);
        m_PIDController.setI(REACHER_K_I);
        m_PIDController.setD(REACHER_K_D);
        m_PIDController.setFF(REACHER_K_FF);

        m_encoder = m_motor.getEncoder();

        // Set the position conversion factor.
        m_encoder.setPositionConversionFactor(REACHER_INCH_PER_REVOLUTION);

        m_encoder.setPosition(REACHER_OFFSET_INCH);

        SmartDashboard.putNumber("Reacher/P Gain", m_kPReacher);
    }

    @Override
    public void periodic() {
        double encoderValue = m_encoder.getPosition();
        SmartDashboard.putNumber("Reacher/Encoder", encoderValue);
        SmartDashboard.putNumber("Reacher/m_goal", m_goal);
        
        super.periodic();

        // update the PID val
        checkPIDVal();
    }

    @Override
    protected void useState(TrapezoidProfile.State setPoint) {
        // Calculate the feedforward from the setPoint
        double feedforward = m_Feedforward.calculate(setPoint.position, setPoint.velocity);

        // Add the feedforward to the PID output to get the motor output
        // Remember that the encoder was already set to account for the gear ratios.

        if (m_resetReacherPos) {
            setPoint.position = m_encoder.getPosition();
            m_resetReacherPos = false;
        }
        m_PIDController.setReference(setPoint.position, ControlType.kPosition, 0); // , feedforward / 12.0);
        SmartDashboard.putNumber("Reacher/setPoint", setPoint.position);
    }

    private void checkPIDVal() {
        double p = SmartDashboard.getNumber("Reacher/P Gain", 0);
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if ((p != m_kPReacher)) {
            m_PIDController.setP(p);
            m_kPReacher = p;
        }
    }

    public double getLength() {
        return m_encoder.getPosition();
    }

    public void resetLength() {
        setGoal(getLength());
        m_resetReacherPos = true;
    }

    public void setBrakeMode(boolean brake) {
        m_motor.setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }

    // set reacher length in inches
    public void setLength(double goal) {
        m_goal = goal;
        super.setGoal(goal);
    }
}
