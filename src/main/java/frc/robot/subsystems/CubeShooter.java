package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;


public class CubeShooter {
    private final CANSparkMax m_motor1;
    private final CANSparkMax m_motor2;
    DigitalInput m_beamBreak = new DigitalInput(0);

    // pneumatic cylinder
    PneumaticHub m_pH = new PneumaticHub(Constants.PNEUMATIC_HUB_CUBE_SHOOTER);
    DoubleSolenoid m_clawSolenoid = m_pH.makeDoubleSolenoid(Constants.DOUBLE_SOLENOID_FORWARD_CHANNEL_CUBE, Constants.DOUBLE_SOLENOID_REVERSE_CHANNEL_CUBE);

    public CubeShooter() {
        m_motor1 = new CANSparkMax(Constants.CUBE_SHOOTER_CAN_ID_1, MotorType.kBrushless);
        m_motor1.restoreFactoryDefaults();    
        m_motor2 = new CANSparkMax(Constants.CUBE_SHOOTER_CAN_ID_2, MotorType.kBrushless);
        m_motor2.restoreFactoryDefaults();  
        SmartDashboard.putBoolean("cubeShooter/isCompressorEnabled", true);
  
    }

    public void periodic(){
        if (SmartDashboard.getBoolean("claw/isCompressorEnabled", true))
            enableCompressor();
        else
            disableCompressor();

    }

    // is there a clear path between sensors
    public boolean isBeamBreak(){
        return m_beamBreak.get();
    }
    // intake a cube
    public void startIntake() {
        m_clawSolenoid.set(Value.kForward);
        //change speed later
        m_motor1.set(-1.0);
    }

    // shoot a cube by inputting desired speed
    public void shootCube(double shooterSpeed) {
        // 2 motors used for shootign cube
        m_motor1.set(shooterSpeed);
        m_motor2.set(shooterSpeed);

    }
    public void enableCompressor() {
        m_pH.enableCompressorDigital();
    }

    public void disableCompressor() {
        m_pH.disableCompressor();
    }
}
