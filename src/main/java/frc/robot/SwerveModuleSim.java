// // package frc.robot;

// // package frc.robot;

// // import edu.wpi.first.math.geometry.Rotation2d;
// // import edu.wpi.first.math.kinematics.SwerveModulePosition;
// // import edu.wpi.first.math.system.plant.DCMotor;
// // import edu.wpi.first.wpilibj.simulation.FlywheelSim;
// // import frc.robot.swerve.NeoDriveController;
// // import frc.robot.swerve.NeoSteerController;

// // // The LigerBots SwerveModule
// // // This has a SteerController and a DriveController


// // public class SwerveModuleSim {

// //     }
// //     private FlywheelSim driveSim = new FlywheelSim(DCMotor.getNEO(4), 6.75, 0.025);
// //     private FlywheelSim SteerSim = new FlywheelSim(DCMotor.getNEO(4), 150.0 / 7.0, 0.004);

// //     public void SwerveModule(FlywheelSim driveController, FlywheelSim steerController) {
// //         driveSim = driveController;
// //         SteerSim = steerController;
// //     }

// //     public double getDriveVelocity() {
// //         return driveSim.getStateVelocity();
// //     }

// //     public Rotation2d getSteerAngle() {
// //         return SteerSim.setstateAngle();
// //     }

// //     public void set(double driveVoltage, double steerAngle) {
// //         steerAngle %= (2.0 * Math.PI);
// //         if (steerAngle < 0.0) {
// //             steerAngle += 2.0 * Math.PI;
// //         }

// //         double currAngle = getSteerAngle().getRadians();
// //         double difference = steerAngle - currAngle;
// //         // Change the target angle so the difference is in the range [-pi, pi) instead
// //         // of [0, 2pi)
// //         if (difference >= Math.PI) {
// //             steerAngle -= 2.0 * Math.PI;
// //         } else if (difference < -Math.PI) {
// //             steerAngle += 2.0 * Math.PI;
// //         }
// //         difference = steerAngle - currAngle; // Recalculate difference

// //         // If the difference is greater than 90 deg or less than -90 deg the drive can
// //         // be inverted so the total
// //         // movement of the module is less than 90 deg
// //         if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
// //             // Only need to add 180 deg here because the target angle will be put back into
// //             // the range [0, 2pi)
// //             steerAngle += Math.PI;
// //             driveVoltage *= -1.0;
// //         }

// //         // Put the target angle back into the range [0, 2pi)
// //         steerAngle %= (2.0 * Math.PI);
// //         if (steerAngle < 0.0) {
// //             steerAngle += 2.0 * Math.PI;
// //         }

// //         driveSim.setReferenceVoltage(driveVoltage);
// //         SteerSim.setReferenceAngle(steerAngle);
// //     }

// //     // get the swerve module position
// //     public SwerveModulePosition getSwerveModulePosition() {
// //         return new SwerveModulePosition(driveSim.getWheelDistance(), SteerSim.getStateAngle());
// //     }



// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.wpilibj.simulation.FlywheelSim;

// public class SwerveModuleSim implements ModuleIO 
// private FlywheelSim driveSim = new FlywheelSim(DCMotor.getNEO(4), 6.75,
// 0.025);
// private FlywheelSim turnSim = new FlywheelSim(DCMotor.getNEO(4), 150.0 / 7.0,
// 0.004);

// private double turnRelativePositionRad = 0.0;
// private double turnAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
// private double driveAppliedVolts = 0.0;
// private double turnAppliedVolts = 0.0;

// public void updateInputs(ModuleIOInputs inputs) {
// driveSim.update(Constants.loopPeriodSecs);
// turnSim.update(Constants.loopPeriodSecs);

// double angleDiffRad = turnSim.getAngularVelocityRadPerSec() *
// Constants.loopPeriodSecs;
// turnRelativePositionRad += angleDiffRad;
// turnAbsolutePositionRad += angleDiffRad;
// while (turnAbsolutePositionRad < 0) {
// turnAbsolutePositionRad += 2.0 * Math.PI;
// }
// while (turnAbsolutePositionRad > 2.0 * Math.PI) {
// turnAbsolutePositionRad -= 2.0 * Math.PI;
// }

// inputs.drivePositionRad =
// inputs.drivePositionRad
// + (driveSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs);
// inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
// inputs.driveAppliedVolts = driveAppliedVolts;
// inputs.driveCurrentAmps = new double[]
// {Math.abs(driveSim.getCurrentDrawAmps())};
// inputs.driveTempCelcius = new double[] {};
// // 
// inputs.turnAbsolutePositionRad = turnAbsolutePositionRad;
// inputs.turnPositionRad = turnRelativePositionRad;
// inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
// inputs.turnAppliedVolts = turnAppliedVolts;
// inputs.turnCurrentAmps = new double[]
// {Math.abs(turnSim.getCurrentDrawAmps())};
// inputs.turnTempCelcius = new double[] {};
// }
// // 
// public void setDriveVoltage(double volts) {
// driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
// driveSim.setInputVoltage(driveAppliedVolts);
// }
// // 
// public void setTurnVoltage(double volts) {
// turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
// turnSim.setInputVoltage(turnAppliedVolts);
// }
