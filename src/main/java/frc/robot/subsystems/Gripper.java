// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.GripperConstants.*;

public class Gripper extends SubsystemBase {
  private final DoubleSolenoid m_grip = new DoubleSolenoid(PN_ID, MODULE_TYPE, FORWARD_CHANNEL, REVERSE_CHANNEL);
  /** Creates a new Gripper. */
  public Gripper() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
