// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveBaseConstants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.commands.ShiftGear;

//import com.github.yehpop.nfsensors.*;


public class DriveBase extends SubsystemBase {
  /** Creates a new DriveBase. */
  private final WPI_VictorSPX m_leftMaster = new WPI_VictorSPX(MOTOR_PORT_1);
  private final WPI_VictorSPX m_leftFollower = new WPI_VictorSPX(MOTOR_PORT_2);
  private final WPI_VictorSPX m_leftThird = new WPI_VictorSPX(MOTOR_PORT_3);
  
  private final WPI_VictorSPX m_rightMaster = new WPI_VictorSPX(MOTOR_PORT_4);
  private final WPI_VictorSPX m_rightFollower = new WPI_VictorSPX(MOTOR_PORT_5);
  private final WPI_VictorSPX m_rightThird = new WPI_VictorSPX(MOTOR_PORT_6);
  
  private final MotorControllerGroup m_leftMotors;
  private final MotorControllerGroup m_rightMotors;

  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);
  
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);
  
  private final AnalogGyro m_gyro = new AnalogGyro(0);
  //private final MPU6050 mpu = new MPU6050();
  
  private final DifferentialDriveOdometry m_odometry;

  // Encoders
  private final Encoder m_leftEncoder = new Encoder(LEFT_ENCODER_PORT_A, LEFT_ENCODER_PORT_B, LEFT_ENCODER_REVERSED, Encoder.EncodingType.k4X); 
  private final Encoder m_rightEncoder = new Encoder(RIGHT_ENCODER_PORT_A, RIGHT_ENCODER_PORT_B, RIGHT_ENCODER_REVERSED, Encoder.EncodingType.k4X); 

  // ShiftGear
  private final DoubleSolenoid m_shifter = new DoubleSolenoid(PN_ID, MODULE_TYPE, FORWARD_CHANNEL, REVERSE_CHANNEL);
  
  private final DifferentialDriveKinematics m_kinematics = 
  new DifferentialDriveKinematics(TRACK_WIDTH);

  public DriveBase() {
    m_leftMotors = new MotorControllerGroup(m_leftMaster, m_leftFollower, m_leftThird);
    m_rightMotors = new MotorControllerGroup(m_rightMaster, m_rightFollower, m_rightThird);
    m_rightMotors.setInverted(true);

    m_leftEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
    m_rightEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);

    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedfoward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedfoward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput = m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
    
    m_leftMotors.setVoltage(leftOutput + leftFeedfoward);
    m_rightMotors.setVoltage(rightOutput + rightFeedfoward);
  }
  
  public void drive(double speed, double rotation) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(speed, 0.0, rotation));
    setSpeeds(wheelSpeeds);
  }
  
  /**
   * Command constructer method.
   *
   * @return command instant command that shifts the gears at the drive base.
   */
  public CommandBase shiftGear() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return this.runOnce(() -> m_shifter.toggle());
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
  
  /**
   * Condition of motion.
   * False; if both sides of drive base are not in motion
   * 
   * @return boolean value.
   */
  public boolean motion() {
    if (m_rightMotors.get() != 0 & m_leftMotors.get() != 0) {
      return true;
    }
    return false;
  }

  /**
   * Condition of the shifter on the drive base.
   * Used to reset shifter when drive base is not in motion.
   * 
   * @return true if shifter is pushed.
   */
  public boolean shifterCondition() {
    if (m_shifter.get() == Value.kReverse) {return true;}
    return false;
  }
  
  ///**
  // * Condition of the shifter on the drive base.
  // * Used to reset shifter when drive base is not in motion.
  // * 
  // * @return true if robot is not in motion and shifter is pushed.
  // */
  //public boolean shifterCondition() {
  //  if (motion() == false & getShifterState() != 1) {return true;}
  //  return false;
  //}
  //
  //public int getShifterState() {
  //  Value state = m_shifter.get(); 
  //  int value;
  //  if (state == Value.kOff) {
  //    value = 0;
  //  } else if (state == Value.kReverse) {
  //    value = -1;
  //  } else if (state == Value.kForward) {
  //    value = 1;
  //  } else { value = 0; }
  //  return value;
  //}

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
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
