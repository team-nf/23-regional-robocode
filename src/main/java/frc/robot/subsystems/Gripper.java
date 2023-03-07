// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.GripperConstants.*;

public class Gripper extends SubsystemBase {
  private final DoubleSolenoid m_grip = new DoubleSolenoid(PN_ID, MODULE_TYPE, FORWARD_CHANNEL, REVERSE_CHANNEL);

  // Neo's
  private final CANSparkMax m_leftDriver = new CANSparkMax(MOTOR_ID_1, MotorType.kBrushless);
  private final CANSparkMax m_rightDriver = new CANSparkMax(MOTOR_ID_2, MotorType.kBrushless);
  private final MotorControllerGroup m_drivers;

  /** Creates a new Gripper. */
  public Gripper() {
    // this should make it so that right is inverted, follows left ansetShiftd i can control them with m_driver
    m_rightDriver.follow(m_leftDriver, true);
    m_drivers = new MotorControllerGroup(m_leftDriver, m_rightDriver);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
