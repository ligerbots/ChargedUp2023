// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(19.5625);

    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(24.625);

    public static final int DRIVETRAIN_NAVX_ID = 0;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 9;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(87.5);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 10;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.5);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(77.3);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(110.2);

    public static final double X_PID_CONTROLLER_P = 0.2;
    public static final double Y_PID_CONTROLLER_P = 0.2;
    public static final double THETA_PID_CONTROLLER_P = 8.0;

    // max voltages
    /**
	 * The maximum voltage that will be delivered to the drive motors.
	 * <p>
	 * This can be reduced to cap the robot's maximum speed. Typically, this is
	 * useful during initial testing of the robot.
	 */

	public static final double MAX_VOLTAGE = 12.0; //default


    // Xbox button mapping
    public static final int XBOX_A = 1;
    public static final int XBOX_B = 2;
    public static final int XBOX_X = 3;
    public static final int XBOX_Y = 4;

    // bumpers
    public static final int XBOX_LB = 5;
    public static final int XBOX_RB = 6;
    
    public static final int XBOX_BACK = 7;
    public static final int XBOX_START = 8;

    // joy stick button
    public static final int XBOX_JL = 9;
    public static final int XBOX_JR = 10;

    // TODO: The following constants came from the 2022 robot.
    // These need to be set for this robot.  


    // Following CAN IDs are for the Arm subsystem
    public static final int ELEVATOR_CAN_ID = 13; // TODO: Set CanID
    public static final int[] SHOULDER_CAN_ID = {14, 15}; // TODO: Set CANIDs
    // Feedforward constants for the each Climber Arm
    public static final double ELEVATOR_KS = 0.182; // TODO: This may need to be tuned
    // The following constants are computed from https://www.reca.lc/arm
    public static final double ELEVATOR_KG = 1.19;
    public static final double ELEVATOR_KV = 7.67;
    public static final double ELEVATOR_KA = 0.19;
    
    // PID Constants for the Arm PID controller
    // Since we're using Trapeziodal control, all values will be 0 except for P
    public static final double ELEVATOR_K_P0 = 100;
    public static final double ELEVATOR_K_P1 = 100;
    public static final double ELEVATOR_K_I = 0.0;
    public static final double ELEVATOR_K_D = 0.0;
    public static final double ELEVATOR_K_FF = 0.0;
    public static final double ELEVATOR_OFFSET_METER = Units.inchesToMeters(1.5);
      // Feedforward constants for the each Climber Arm
      public static final double ARM_KS = 0.182; // TODO: This may need to be tuned
      // The following constants are computed from https://www.reca.lc/arm
      public static final double ARM_KG = 2.07;
      public static final double ARM_KV = 1.83;
      public static final double ARM_KA = 0.08;
  
      // Constants to limit the arm rotation speed
      public static final double ARM_MAX_VEL_RAD_PER_SEC = Math.toRadians(200.0);
      public static final double ARM_MAX_ACC_RAD_PER_SEC_SQ = Math.toRadians(200.0);
      public static final double ARM_OFFSET_RAD = Math.toRadians(110.0);
      
      // PID Constants for the Arm PID controller
      // Since we're using Trapeziodal control, all values will be 0 except for P
      public static final double ARM_K_P = 10.0;
      public static final double ARM_K_I = 0.0;
      public static final double ARM_K_D = 0.0;
      public static final double ARM_K_FF = 0.0;
      public static final int kPIDLoopIdx = 0;
      public static final double ARM_ANGLE_TOLERANCE = Units.degreesToRadians(1.0);
      public static final int kTimeoutMs = 0;
          // Constants to limit the elevator veocity and accel

    public static final double ELEVATOR_MAX_VEL_METER_PER_SEC_ASCEND = Units.inchesToMeters(1000.0);
    public static final double ELEVATOR_MAX_ACC_METER_PER_SEC_SQ_ASCEND = Units.inchesToMeters(250.0);

    public static final double ELEVATOR_MAX_VEL_METER_PER_SEC_DESCEND = Units.inchesToMeters(100.0);
    public static final double ELEVATOR_MAX_ACC_METER_PER_SEC_SQ_DESCEND = Units.inchesToMeters(30.0);

    public static final double ELEVATOR_MAX_VEL_METER_PER_SEC_NORMAL = Units.inchesToMeters(50.0);
    public static final double ELEVATOR_MAX_ACC_METER_PER_SEC_SQ_NORMAL = Units.inchesToMeters(10.0);

    public static final double ELEVATOR_MAX_VEL_METER_PER_SEC_ASCEND_SLOW = Units.inchesToMeters(1000.0);
    public static final double ELEVATOR_MAX_ACC_METER_PER_SEC_SQ_ASCEND_SLOW = Units.inchesToMeters(62.5);

       // Constants for reaching the top level, middle level, bottom level, and feeder station. Currently just have placeholder values

    public static final double ARM_TOP_ROW = Math.toRadians(90);
    public static final double ARM_MIDDLE_ROW = Math.toRadians(0.0);
    public static final double ARM_BOTTOM_ROW = Math.toRadians(0.0);
    public static final double FEEDER_STATION = Math.toRadians(0.0);

        


}
