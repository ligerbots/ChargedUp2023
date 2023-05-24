package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;


public class CubeShooter {
    private final CANSparkMax m_cubeShooterMotor;
    

    public CubeShooter() {
        // motor for cube shooting
        m_cubeShooterMotor = new CANSparkMax(Constants.CUBE_SHOOTER_CAN_ID, MotorType.kBrushless);
        m_cubeShooterMotor.restoreFactoryDefaults();    
    }

    // intake a cube
    public void startIntake() {
        //change speed later
        m_cubeShooterMotor.set(-1.0);
    }

    // shoot a cube by inputting desired speed
    public void shootCube(double shooterSpeed) {
        m_cubeShooterMotor.set(shooterSpeed);
    }
}
