package frc.robot.subsystems;

import edu.wpi.first.wpilibj.simulation.RoboRioSim;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

// Simulation of the Shoulder
// This uses Falcons, so it is a little specific to that.
// Does not yet account for the varying length of the arm.

public class ShoulderSim {
    private final TalonFXSimCollection m_motorSim;
    private final WPI_TalonFX m_motor;
    private final double m_radiansPerTick;

    // The arm gearbox represents a gearbox containing two Falcon motors.
    private final DCMotor m_shoulderGearbox = DCMotor.getFalcon500(2);

    // Simulation classes help us simulate what's going on, including gravity.
    private static final double SHOULDER_MASS = 10; // Kilograms
    // should use an average or typical length, because it is fixed in the simulation
    private static final double SHOULDER_LENGTH = Units.inchesToMeters(30);

    private final SingleJointedArmSim m_shoulderSim;

    // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
    private final Mechanism2d m_mech2d;
    private final MechanismLigament2d m_arm;

    // for now, really a shoulder simulation
    public ShoulderSim(WPI_TalonFX motor, double radiansPerTick) {
        m_motor = motor;
        m_motorSim = motor.getSimCollection();
        m_radiansPerTick = radiansPerTick;

        // This is the Physics simulation of an arm
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
        double motorVolts = m_motor.getMotorOutputVoltage();
        SmartDashboard.putNumber("shoulder/simVolts", motorVolts);
        m_shoulderSim.setInput(motorVolts);

        // Next, we update it. The standard loop time is 20ms.
        m_shoulderSim.update(0.020);

        // Now, update the simulated encoders to match
        double angle = m_shoulderSim.getAngleRads();
        m_motorSim.setIntegratedSensorRawPosition((int)(angle / m_radiansPerTick));
        m_motorSim.setIntegratedSensorVelocity((int)(m_shoulderSim.getVelocityRadPerSec() / m_radiansPerTick));

        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_shoulderSim.getCurrentDrawAmps()));
        m_motorSim.setBusVoltage(RobotController.getBatteryVoltage());

        // Update the Mechanism Arm angle based on the simulated arm angle
        m_arm.setAngle(Math.toDegrees(angle));
    }
}
