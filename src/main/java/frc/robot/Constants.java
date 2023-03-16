// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    // Controller Ports
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final int DRIVE_MODE = 0;
    public static final boolean TEST = true;
    public static final boolean OPERATING = false;
  }
  public static class CAN {
    public static final int PDH_ID = 1;
    public static final int REVPH = 1;
    public static final int[] SPARK = {2, 5, 6, 7, 3, 4};
    public static final int[] VICTOR = {8, 9, 10, 11, 12, 13};
  }
  public static enum PIDCoefficients {
    DriveBase(
      0,
      1,
      0,
      0,
      0,
      0.5,
      1,
      -1,
      330
    ),
    Arm(
    0,
    3,
    1e-2,
    0.5, 
    0, 
    0.000156, 
    1, 
    -1,
    240, 
    180, // rpm
    -180,
    10, 
    5,
    1,
    1,
    0.5,
    0.1
    ),
    Wrist(
    0,
    3,
    1e-2,
    0.5, 
    0, 
    0.000156, 
    1, 
    -1,
    240, 
    180, // rpm
    -180,
    10, 
    5,
    1,
    1,
    0.5,
    0.1
    ),
    Turret(
    0,
    5e-5,
    1e-6,
    0, 
    0, 
    0.000156, 
    1, 
    -1,
    330,
    330, // rpm
    -330,
    12,
    0.04,
    1,
    1,
    0.5,
    0.1
    );
    public final int PID_SLOT;
    public final double P, I, D, IZ, FF, MAX_OUTPUT, MIN_OUTPUT, MAX_RPM, MAX_VEL, MIN_VEL, MAX_ACC, ALLOWED_ERR, S, G, V, A; 
    private PIDCoefficients(int slot, double p, double i, double d, double iz, double ff, double max_out, double min_out, double rpm) {
      this.PID_SLOT = slot;
      this.P = p;
      this.I = i;
      this.D = d;
      this.IZ = iz;
      this.FF = ff;
      this.MAX_OUTPUT = max_out;
      this.MIN_OUTPUT = min_out;
      this.MAX_RPM = rpm;
      this.MAX_VEL = 0;
      this.MIN_VEL = 0;
      this.MAX_ACC = 0;
      this.ALLOWED_ERR = 0;
      this.S = 0;
      this.G = 0;
      this.V = 0;
      this.A = 0;
    }
    private PIDCoefficients(int slot, double p, double i, double d, double iz, double ff, double max_out, double min_out, double rpm, double max_vel, double min_vel, double acc, double err){
      this.PID_SLOT = slot;
      this.P = p;
      this.I = i;
      this.D = d;
      this.IZ = iz;
      this.FF = ff;
      this.MAX_OUTPUT = max_out;
      this.MIN_OUTPUT = min_out;
      this.MAX_RPM = rpm;
      this.MAX_VEL = max_vel;
      this.MIN_VEL = min_vel;
      this.MAX_ACC = acc;
      this.ALLOWED_ERR = err;
      this.S = 0;
      this.G = 0;
      this.V = 0;
      this.A = 0;
    }
    private PIDCoefficients(int slot, double p, double i, double d, double iz, double ff, double max_out, double min_out, double rpm, double max_vel, double min_vel, double acc, double err, double s, double g, double v, double a){
      this.PID_SLOT = slot;
      this.P = p;
      this.I = i;
      this.D = d;
      this.IZ = iz;
      this.FF = ff;
      this.MAX_OUTPUT = max_out;
      this.MIN_OUTPUT = min_out;
      this.MAX_RPM = rpm;
      this.MAX_VEL = max_vel;
      this.MIN_VEL = min_vel;
      this.MAX_ACC = acc;
      this.ALLOWED_ERR = err;
      this.S = s;
      this.G = g;
      this.V = v;
      this.A = a;
    }
  }
  public static class DriveBaseConstants {
    // Mathematics
    public static final double SPEED = 10.0;  // meters per second
    public static final double MAX_ANGULAR_VELOCITY = 2 * Math.PI; // one rotation per second
    public static final double TRACK_WIDTH = 0.605; // meters
    public static final double WHEEL_RADIUS = 0.07; // meters
    public static final PIDCoefficients PID = PIDCoefficients.DriveBase;

    // Motor Ports
    public static final int MOTOR_PORT_1 = CAN.VICTOR[0];
    public static final int MOTOR_PORT_2 = CAN.VICTOR[1];
    public static final int MOTOR_PORT_3 = CAN.VICTOR[2];
    public static final int MOTOR_PORT_4 = CAN.VICTOR[3];
    public static final int MOTOR_PORT_5 = CAN.VICTOR[4];
    public static final int MOTOR_PORT_6 = CAN.VICTOR[5];
        
    // Encoders
    public static final int LEFT_ENCODER_PORT_A = 0;
    public static final int LEFT_ENCODER_PORT_B = 1;
    public static final int RIGHT_ENCODER_PORT_A = 2;
    public static final int RIGHT_ENCODER_PORT_B = 3;
    public static final boolean LEFT_ENCODER_REVERSED = false;
    public static final boolean RIGHT_ENCODER_REVERSED = true;
    public static final boolean LEFT_MOTORS_REVERSED = false;
    public static final boolean RIGHT_MOTORS_REVERSED = true;

    // Switches
    public static final int LIMIT_CH = 4;
    
    public static final double ENCODER_RESOLUTION = 400; // 1X
    public static final double DISTANCE_PER_PULSE = 2 * Math.PI * WHEEL_RADIUS / ENCODER_RESOLUTION;

    // Pneumatics
    public static final PneumaticsModuleType MODULE_TYPE = PneumaticsModuleType.REVPH;
    public static final int PN_ID = CAN.REVPH;
    public static final int FORWARD_CHANNEL = 0;
    public static final int REVERSE_CHANNEL = 1;
  }
  public static class TurretConstants {
    public static final int MOTOR_ID = CAN.SPARK[0];
    public static final int LIMIT_CH = 5;
    
    public static final double GEAR_RATE = 60.0/14.0*48.0/20.0*200.0/24.0;
    //public static final double ENCODER_RESOLUTION = 42;
    public static final double ENCODER_CPR = 42;
    public static final double ENCODER_START = 0;

    //public static final double DISTANCE_PER_PULSE = 360. / (GEAR_RATE * ENCODER_RESOLUTION);
    public static final double DISTANCE_PER_COUNT = 360. / (GEAR_RATE * ENCODER_CPR);
    public static final double DISTANCE_PER_REV = 360. / GEAR_RATE;

    public static final PIDCoefficients COEFF = PIDCoefficients.Turret;

    public static final boolean MANUAL = OperatorConstants.OPERATING;
  }
  public static class LiftConstants {
    public static final double GEAR_RATE = (50.0/14.0)*(48.0/16.0);
    public static final double GEAR_RADIUS = .0035/2.0;
    
    public static final int MOTOR_ID = CAN.SPARK[1];
    public static final int LIMIT_CH_1 = 6;
    public static final int LIMIT_CH_2 = 7;
    
    //public static final double ENCODER_RESOLUTION = 42;
    public static final double ENCODER_CPR = 42;
    /**meters */
    //public static final double DISTANCE_PER_PULSE = (2 * Math.PI * GEAR_RADIUS) / (ENCODER_RESOLUTION * GEAR_RATE);
    /**meters */
    public static final double DISTANCE_PER_REV = (2 * Math.PI * GEAR_RADIUS) / GEAR_RATE;    
    /**meters */
    public static final double DISTANCE_PER_COUNT = (2 * Math.PI * GEAR_RADIUS) / (ENCODER_CPR * GEAR_RATE);
  }
  public static class CarriageConstants {
    public static final double GEAR_RATE = 36.0/12.0*56.0/18.0*56.0/18.0*36.0/12.0;
    //public static final double ENCODER_RESOLUTION = 42;
    public static final double ENCODER_CPR = 42;

    // SET
    public static final double WRIST_START = 0.0;
    public static final double ARM_START = 0.0;

    //public static final double DISTANCE_PER_PULSE = 360. / (GEAR_RATE * ENCODER_RESOLUTION);
    public static final double DISTANCE_PER_COUNT = 360. / (GEAR_RATE * ENCODER_CPR);
    public static final double DISTANCE_PER_REV = 360. / GEAR_RATE;
    
    public static final int MOTOR_ID_1 = CAN.SPARK[2];
    public static final int MOTOR_ID_2 = CAN.SPARK[3];
    
    // Pneumatics
    public static final PneumaticsModuleType MODULE_TYPE = PneumaticsModuleType.REVPH;
    public static final int PN_ID = CAN.REVPH;
    public static final int FORWARD_CHANNEL_1 = 2;
    public static final int REVERSE_CHANNEL_1 = 3;
    public static final int FORWARD_CHANNEL_2 = 4;
    public static final int REVERSE_CHANNEL_2 = 5;

    // PID Coefficients
    public static final PIDCoefficients ARM_PID = PIDCoefficients.Arm;
    public static final PIDCoefficients WRIST_PID = PIDCoefficients.Wrist;
    public static final boolean MANUAL = OperatorConstants.OPERATING;

    // Max Velocity Degrees Per Second, Max Acceleration Degrees Per Second Squared
    public static final double[] CONSTRAINTS = {45, 57.2957795130931}; 
  }
  public static class GripperConstants {
    // Motors
    public static final int MOTOR_ID_1 = CAN.SPARK[4];
    public static final int MOTOR_ID_2 = CAN.SPARK[5];

    public static final double INTAKE_SPEED = 0.5;
    public static final double SHOOTER_SPEED = 0.3;
    
    // Pneumatics
    public static final PneumaticsModuleType MODULE_TYPE = PneumaticsModuleType.REVPH;
    public static final int PN_ID = CAN.REVPH;
    public static final int FORWARD_CHANNEL = 6;
    public static final int REVERSE_CHANNEL = 7;
  }
  public static class TestConstants {
    // Drive Base
    public static final double TEST_CHASSIS_SPEED = 1.0;
    public static final double TEST_CHASSIS_ANGULAR = 2 * Math.PI;
    
    // Turret

    // Lift
    public static final double TEST_LIFT_SPEED = 0.1;

    // Carriage

    // Gripper
    public static final double TEST_INTAKE_SPEED = 0.6;
    public static final double TEST_SHOOTER_SPEED = 0.2;
  }
}
