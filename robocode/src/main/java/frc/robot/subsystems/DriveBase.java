// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import static frc.robot.Constants.DriveBaseConstants.*;

public class DriveBase extends SubsystemBase {
  /** Creates a new DriveBase. */
  private final WPI_VictorSPX m_leftMaster = new WPI_VictorSPX(MOTOR_PORT_1);
  private final WPI_VictorSPX m_leftFollower = new WPI_VictorSPX(MOTOR_PORT_2);
  
  private final WPI_VictorSPX m_rightMaster = new WPI_VictorSPX(MOTOR_PORT_3);
  private final WPI_VictorSPX m_rightFollower = new WPI_VictorSPX(MOTOR_PORT_4);
  
  public final MotorControllerGroup m_leftMotors;
  public final MotorControllerGroup m_rightMotors;

  public DriveBase() {
    m_leftMotors = new MotorControllerGroup(m_leftMaster, m_leftFollower);
    m_rightMotors = new MotorControllerGroup(m_rightMaster, m_rightFollower);
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
