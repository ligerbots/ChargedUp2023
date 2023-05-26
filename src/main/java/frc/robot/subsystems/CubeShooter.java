package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class CubeShooter extends SubsystemBase {
    private final CANSparkMax m_intakeMotor;
    private final CANSparkMax m_shootMotor;

    //change speed later
    private final double INTAKE_SPEED = 0.3;

    DigitalInput m_beamBreak = new DigitalInput(0);

    DoubleSolenoid m_deploySolenoid;

    public CubeShooter(PneumaticHub pneuHub) {
        // motor for cube shooting
        m_intakeMotor = new CANSparkMax(Constants.CUBE_SHOOTER_INTAKE_CAN_ID, MotorType.kBrushless);
        m_shootMotor = new CANSparkMax(Constants.CUBE_SHOOTER_SHOOT_CAN_ID, MotorType.kBrushless);

        m_intakeMotor.restoreFactoryDefaults();
        m_shootMotor.restoreFactoryDefaults();     
        m_shootMotor.setInverted(true);

        m_deploySolenoid = pneuHub.makeDoubleSolenoid(Constants.CUBE_SHOOTER_CYLINDER_FORWARD, Constants.CUBE_SHOOTER_CYLINDER_REVERSE);
    }

    // is there a clear path between sensors
    public boolean isBeamBreak(){
        return m_beamBreak.get();
    }
     
    // deploy or retract
    public void deployIntake(boolean deploy) {
        if (deploy) {
            // set shooter out
            m_deploySolenoid.set(Value.kForward);
        }
        else
        {
            // retract
            setSpeed(0);
            m_deploySolenoid.set(Value.kReverse);
        }
    }

    // intake a cube
    public void startIntake() {
        //change speed later
        m_intakeMotor.set(INTAKE_SPEED);
    }

    // shoot a cube by inputting desired speed
    public void setSpeed(double shooterSpeed) {
        m_intakeMotor.set(shooterSpeed);
        m_shootMotor.set(shooterSpeed);
    }
}
