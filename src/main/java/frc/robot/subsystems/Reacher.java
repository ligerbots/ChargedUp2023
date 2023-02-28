// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;

public class Reacher extends TrapezoidProfileSubsystem {

    private static final double REACHER_MAX_LENGTH = Units.inchesToMeters(35.0);
    private static final double REACHER_MIN_LENGTH = Units.inchesToMeters(0.5);

    public static final double REACHER_OFFSET_TOLERANCE_METERS = Units.inchesToMeters(1.0);

    // Constants to limit the shoulder rotation speed
    // For initial testing, these should be very slow.
    // We can update them as we get more confidence.
    private static final double REACHER_MAX_VEL_METER_PER_SEC = Units.inchesToMeters(100.0);
    // Let's give it 2 seconds to get to max velocity.
    // Once tuned, I expect we will want this to be equal to REACHER_MAX_VEL_METER_PER_SEC
    // so it will get to max velocity in one second.
    private static final double REACHER_MAX_ACC_METER_PER_SEC_SQ = Units.inchesToMeters(100.0);

    // TODO: See if this is close enough. Or do we need a more exact measurement?
    private static final double REACHER_METER_PER_REVOLUTION = Units.inchesToMeters(0.7);

    private static final double REACHER_OFFSET_METER = Units.inchesToMeters(0.0);



    // Feedforward constants for the reacher
    private static final double REACHER_KS = 0.182; // TODO: This may need to be tuned
    // The following constants are computed from https://www.reca.lc/linear
    private static final double REACHER_KG = 0.16; // Volts
    private static final double REACHER_KV = 3.07; // V*sec/meter
    private static final double REACHER_KA = 0.03; // V*sec^2/meter

    // PID Constants for the reacher PID controller
    // Since we're using Trapeziodal control, all values will be 0 except for P
    private static final double REACHER_K_P = 10.0;
    // private static final double REACHER_K_P1 = 100;
    private static final double REACHER_K_I = 0.0;
    private static final double REACHER_K_D = 0.0;
    private static final double REACHER_K_FF = 0.0;

    // Define the motor and encoders
    private final CANSparkMax m_motor;
    private final RelativeEncoder m_encoder;
    private final SparkMaxPIDController m_PIDController;

    private final ElevatorFeedforward m_feedForward = new ElevatorFeedforward(REACHER_KS,
                    REACHER_KG, REACHER_KV, REACHER_KA);

    private double m_kPReacher;
    private boolean m_resetReacherPos = false;
    private boolean m_coastMode = false;
    private double m_goal = 0;

    /** Creates a new Reacher. */
    public Reacher() {
        super(new TrapezoidProfile.Constraints(REACHER_MAX_VEL_METER_PER_SEC, REACHER_MAX_ACC_METER_PER_SEC_SQ));

        m_kPReacher = REACHER_K_P;

        // Create the motor, PID Controller and encoder.
        m_motor = new CANSparkMax(Constants.REACHER_CAN_ID, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
        
        //set currentLimit for reacher to 35 amps
        m_motor.setSmartCurrentLimit(35);

        m_PIDController = m_motor.getPIDController();
        m_PIDController.setP(REACHER_K_P);
        m_PIDController.setI(REACHER_K_I);
        m_PIDController.setD(REACHER_K_D);
        m_PIDController.setFF(REACHER_K_FF);
        // m_motor.setInverted(true);

        m_encoder = m_motor.getEncoder();

        // Set the position conversion factor.
        m_encoder.setPositionConversionFactor(REACHER_METER_PER_REVOLUTION);

        m_encoder.setPosition(REACHER_OFFSET_METER);

        setCoastMode(false);
        SmartDashboard.putBoolean("Reacher/m_coastMode", m_coastMode);

        SmartDashboard.putNumber("Reacher/P Gain", m_kPReacher);
    }

    @Override
    public void periodic() {
        double encoderValue = m_encoder.getPosition();
        SmartDashboard.putNumber("Reacher/Encoder", Units.metersToInches(encoderValue));
        SmartDashboard.putNumber("Reacher/m_goal", Units.metersToInches(m_goal));
        SmartDashboard.putBoolean("Reacher/m_resetReacherPos", m_resetReacherPos);
        
        m_coastMode = SmartDashboard.getBoolean("Reacher/m_coastMode", m_coastMode);
        if(m_coastMode)
            setCoastMode(m_coastMode);
        
        // Jack: I dont think we need this check because we are only going to set to coast mode during disabled and the motor won't be moved by super.periodic() or useState() anyway
        // And we want the setPoint to follow the current encoder reading in disabled mode
        // if (m_coastMode)
        //     return;
        super.periodic();

        // update the PID val
        checkPIDVal();
    }

    @Override
    protected void useState(TrapezoidProfile.State setPoint) {
        // Calculate the feedforward from the setPoint
        double feedforward = m_feedForward.calculate(setPoint.position, setPoint.velocity);

        // Add the feedforward to the PID output to get the motor output
        // Remember that the encoder was already set to account for the gear ratios.

        if (m_resetReacherPos) {
            setPoint.position = m_encoder.getPosition();
            m_resetReacherPos = false;
        }
        m_PIDController.setReference(setPoint.position, ControlType.kPosition, 0); // , feedforward / 12.0);
        SmartDashboard.putNumber("Reacher/setPoint", Units.metersToInches(setPoint.position));
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

    public void resetReacherPos() {
        setLength(getLength());
        m_resetReacherPos = true;
    }

    public static double limitReacherLength(double length){
        return Math.min(REACHER_MAX_LENGTH, Math.max(REACHER_MIN_LENGTH, length));
    }

    // set reacher length in inches
    public void setLength(double goal) {
        m_goal = limitReacherLength(goal);
        super.setGoal(m_goal);
    }

    public void resetGoal(){
        setLength(getLength());
    }

    public void setCoastMode(boolean coastMode){
        if(coastMode){
            m_motor.setIdleMode(IdleMode.kCoast);
            m_motor.stopMotor();
        }else
            m_motor.setIdleMode(IdleMode.kBrake);
    }
}
