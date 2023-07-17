package frc.robot.subsystems;

import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ShoulderSim {
    private final EncoderSim m_encoderSim;
    private final MotorController m_shoulderMotor;
    
    // // The arm gearbox represents a gearbox containing two Falcon motors.
    private final DCMotor m_shoulderGearbox = DCMotor.getFalcon500(2);

    // // Simulation classes help us simulate what's going on, including gravity.
    private static final double SHOULDER_MASS = 10; // Kilograms
    private static final double SHOULDER_LENGTH = Units.inchesToMeters(30);

    // private TalonFXSimCollection m_motorSim;

    private final SingleJointedArmSim m_shoulderSim;

    // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
    private final Mechanism2d m_mech2d;
    private final MechanismLigament2d m_arm;

    // for now, really a shoulder simulation
    public ShoulderSim(MotorController shoulderMotor, EncoderSim shoulderEncoder) {
        m_shoulderMotor = shoulderMotor;
        m_encoderSim = shoulderEncoder;

        m_shoulderSim = new SingleJointedArmSim(m_shoulderGearbox, Shoulder.SHOULDER_GEAR_RATIO,
                SingleJointedArmSim.estimateMOI(SHOULDER_LENGTH, SHOULDER_MASS), SHOULDER_LENGTH,
                Units.degreesToRadians(-65), Units.degreesToRadians(120), true);

        // Build the Mechanism2d so it can be displayed in Glass
        m_mech2d = new Mechanism2d(60, 60);
        MechanismRoot2d shoulderPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
        shoulderPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
        m_arm = shoulderPivot.append(new MechanismLigament2d("Arm", 30,
                Math.toDegrees(m_shoulderSim.getAngleRads()), 6, new Color8Bit(Color.kYellow)));

        SmartDashboard.putData("ArmSimulation", m_mech2d);
    }

    // this is not called automatically, since it is not a Subsystem
    public void simulationPeriodic() {
        // First, we set our "inputs" (voltages)
        double motorSpeed = m_shoulderMotor.get();
        SmartDashboard.putNumber("shoulder/simSpeed", motorSpeed);
        m_shoulderSim.setInput(motorSpeed * RobotController.getBatteryVoltage());
        // Next, we update it. The standard loop time is 20ms.
        m_shoulderSim.update(0.020);
        double angle = m_shoulderSim.getAngleRads();

        // set the simulated encoder to match
        m_encoderSim.setDistance(angle);

        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_shoulderSim.getCurrentDrawAmps()));

        // Update the Mechanism Arm angle based on the simulated arm angle
        m_arm.setAngle(Math.toDegrees(angle));
    }

    public double getArmAngle() {
        return Math.toRadians(m_arm.getAngle());
    }
}
