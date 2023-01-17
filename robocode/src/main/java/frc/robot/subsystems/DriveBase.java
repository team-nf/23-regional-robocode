// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveBaseConstants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase {
  /** Creates a new DriveBase. */
  private final WPI_VictorSPX m_leftMaster = new WPI_VictorSPX(MOTOR_PORT_1);
  private final WPI_VictorSPX m_leftFollower = new WPI_VictorSPX(MOTOR_PORT_2);
  
  private final WPI_VictorSPX m_rightMaster = new WPI_VictorSPX(MOTOR_PORT_3);
  private final WPI_VictorSPX m_rightFollower = new WPI_VictorSPX(MOTOR_PORT_4);
  
  public final MotorControllerGroup m_leftMotors;
  public final MotorControllerGroup m_rightMotors;

  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);
  
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);
  
  private final AnalogGyro m_gyro = new AnalogGyro(0);
  private final DifferentialDriveOdometry m_odometry;
  
  // Encoders
  private final Encoder m_leftEncoder = new Encoder(LEFT_ENCODER_PORT_A, LEFT_ENCODER_PORT_B, LEFT_ENCODER_REVERSED, Encoder.EncodingType.k4X); 
  private final Encoder m_rightEncoder = new Encoder(RIGHT_ENCODER_PORT_A, RIGHT_ENCODER_PORT_B, RIGHT_ENCODER_REVERSED, Encoder.EncodingType.k4X); 
  
  private final DifferentialDriveKinematics m_kinematics = 
  new DifferentialDriveKinematics(TRACK_WIDTH);

  public DriveBase() {
    m_leftMotors = new MotorControllerGroup(m_leftMaster, m_leftFollower);
    m_rightMotors = new MotorControllerGroup(m_rightMaster, m_rightFollower);

    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update Odometry Pose
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // For testing purposes
    m_leftMotors.set(0.5);
    m_rightMotors.set(0.2);

    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

    System.out.println(m_odometry.getPoseMeters());
  }
}
