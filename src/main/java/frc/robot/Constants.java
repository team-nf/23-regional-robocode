// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int kDriverControllerPort = 0;
  }
  public static class DriveBaseConstants {
    // Mathematics
    public static final double SPEED = 10.0;  // meters per second
    public static final double MAX_ANGULAR_VELOCITY = 2 * Math.PI; // one rotation per second
    public static final double TRACK_WIDTH = 0.605; // meters
    public static final double WHEEL_RADIUS = 0.07; // meters

    // Motor Ports
    public static final int MOTOR_PORT_1 = 0;
    public static final int MOTOR_PORT_2 = 1;
    public static final int MOTOR_PORT_3 = 2;
    public static final int MOTOR_PORT_4 = 3;
    
    // Encoders
    public static final int LEFT_ENCODER_PORT_A = 0;
    public static final int LEFT_ENCODER_PORT_B = 1;
    public static final int RIGHT_ENCODER_PORT_A = 2;
    public static final int RIGHT_ENCODER_PORT_B = 3;
    public static final boolean LEFT_ENCODER_REVERSED = false;
    public static final boolean RIGHT_ENCODER_REVERSED = true;
    public static final boolean LEFT_MOTORS_REVERSED = false;
    public static final boolean RIGHT_MOTORS_REVERSED = true;
    
    public static final double ENCODER_RESOLUTION = 400; // 1X
    public static final double DISTANCE_PER_PULSE = 2 * Math.PI * WHEEL_RADIUS / ENCODER_RESOLUTION;

  }
}
