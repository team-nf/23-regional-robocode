// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class AutonomousConstants {
    public static enum Auto {
      SIMPLE,
      CHARGED;

      public static enum Position {
        TOP,
        CENTER,
        BOTTOM
      }
      private static final Position START_POSITION = Position.TOP;
      public Position pos() {
        return START_POSITION;
      }
    } 
    public static final Auto AUTO = Auto.SIMPLE;

    public static final HashMap<String, Command> eventMap = new HashMap<>();
  }
  public static class OperatorConstants {
    // Controller Ports
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final int DRIVE_MODE = 0;
    public static final boolean TEST = true;
    public static final boolean OPERATING = false;
    
    //*Canceled. The SmartDashboard tab on Shuffleboard is used for robot configurations. */
    public static final ShuffleboardTab CONFIG_DASHBOARD = Shuffleboard.getTab("Configuration"); 
    public static final ShuffleboardTab DRIVER_DASHBOARD = Shuffleboard.getTab("CompetitionDashboard");
  }
  public static class CAN {
    public static final int PDH_ID = 1;
    public static final int REVPH = 1;
    public static final int[] SPARK = {7, 4, 2, 6, 3, 5};
    public static final int[] VICTOR = {8, 9, 10, 11, 12, 13};
  }
  public static enum PIDCoefficients {
    DriveBase(
      new HashMap<String, Double>(){
        {
        put("p", 1.0);
        put("i", 0.0);
        put("d", 0.0);
        put("maxAcc", 3.0);
        put("maxOut", 1.0);
        put("minOut", -1.0);
        put("s", 0.004);
        put("v", 0.008);
        }
      }
    ),
    Turret(
      new HashMap<String, Double>() {
        {
        put("slot", 1.0);
        put("p", 0.0030479);
        put("i", 0.0);
        put("d", 0.00095926);
        put("iz", 0.0);
        put("ff", 0.14);
        put("maxOut", 0.2);
        put("minOut", -0.2);
        put("rpm", 330.0);
        put("maxVel", 30.0);
        put("minVel", -30.0);
        put("maxAcc", 12.0);
        put("allowedErr", 0.52375);
        put("s", 0.063526);
        put("v", 0.5592);
        put("a", 0.1222);
        }
      }
    ),
    Arm(
      new HashMap<String, Double>() {
        {
        put("p", 0.0051415);
        put("i", 0.0);
        put("d", 0.001146);
        put("iz", 0.0);
        put("ff", 0.0);
        put("maxOut", 0.4);
        put("minOut", -0.4);
        put("rpm", 240.0);
        put("maxVel", 180.0);
        put("minVel", -180.0);
        put("maxAcc", 10.0);
        put("allowedErr", 0.125);
        put("s", 0.10922);
        put("g", 0.0089398);
        put("v", 0.37174);
        put("a", 0.014984);
        }
      }
    ),
    Wrist(
      new HashMap<String, Double>() {
        {
          put("p", 0.0051415);
          put("i", 0.0);
          put("d", 0.001146);
          put("iz", 0.0);
          put("ff", 0.0);
          put("maxOut", 0.4);
          put("minOut", -0.4);
          put("rpm", 240.0);
          put("maxVel", 180.0);
          put("minVel", -180.0);
          put("maxAcc", 10.0);
          put("allowedErr", 0.125);
          put("s", 0.10922);
          put("g", 0.0089398);
          put("v", 0.37174);
          put("a", 0.014984);
          }
      }
    ),
    Lift(
      new HashMap<String, Double>() {
        {
          put("g", -0.066316);
          put("s", 0.29854);
          put("v", 1.0);
          put("a", 5.2737);
          put("p", 3.2);
          put("d", 1.98);
          put("allowedErr", 0.085);
          put("maxOut", 0.3);
          put("minOut", -0.3);
        }
      }
    );
    public final int PID_SLOT;
    public final double P, I, D, IZ, FF, MAX_OUTPUT, MIN_OUTPUT, MAX_RPM, MAX_VEL, MIN_VEL, MAX_ACC, ALLOWED_ERR, S, G, V, A; 
    
    static double getParm(Map<String, Double> map, String key, double defaultValue) {
      return (map.containsKey(key)) ? (double) map.get(key) : defaultValue;
    }
    private PIDCoefficients(Map<String, Double> coeffs) {
      this.PID_SLOT = (int) getParm(coeffs, "slot", 0);
      this.P = getParm(coeffs, "p", 0);
      this.I = getParm(coeffs, "i", 0);
      this.D = getParm(coeffs, "d", 0);
      this.IZ = getParm(coeffs, "iz", 0);
      this.FF = getParm(coeffs, "ff", 0);
      this.MAX_OUTPUT = getParm(coeffs, "maxOutput", 1);
      this.MIN_OUTPUT = getParm(coeffs, "minOutput", -1);
      this.MAX_RPM = getParm(coeffs, "rpm", 660);
      this.MAX_VEL = getParm(coeffs, "maxVel", 0);
      this.MIN_VEL = getParm(coeffs, "minVel", 0);
      this.MAX_ACC = getParm(coeffs, "maxAcc", 0);
      this.ALLOWED_ERR = getParm(coeffs, "allowedErr", 0);
      this.S = getParm(coeffs, "s", 0);
      this.G = getParm(coeffs, "g", 0);
      this.V = getParm(coeffs, "v", 0);
      this.A = getParm(coeffs, "a", 0);
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
    public static final boolean LEFT_ENCODER_REVERSED = true;
    public static final boolean RIGHT_ENCODER_REVERSED = false;
    public static final boolean LEFT_MOTORS_REVERSED = true;
    public static final boolean RIGHT_MOTORS_REVERSED = false;

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
    public static final double TEST_CHASSIS_SPEED = 3.0;
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
