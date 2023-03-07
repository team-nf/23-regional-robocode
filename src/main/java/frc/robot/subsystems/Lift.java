// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.LiftConstants.*;

public class Lift extends SubsystemBase {
  // Neo550
  private final CANSparkMax m_driver = new CANSparkMax(1, MotorType.kBrushless);

  // Limit Switches
  private final DigitalInput m_topLimit = new DigitalInput(0);
  private final DigitalInput m_bottomLimit = new DigitalInput(1);
  
  /** Creates a new Lift. */
  public Lift() {}

  /**
   * Condition method.
   * Panic if limit switch is hit.
   * Used for the break.
   * 
   * @return True for panic, false for OK.
   */
  public boolean panicCondition() {
    if(m_topLimit.get()) {m_driver.set(0);}
    if(m_bottomLimit.get()) {m_driver.set(0);}
    return false;
  }

  /**
   * Condition method
   * 
   * @return True if lift is in motion, false if stationary.
   */
  public boolean motion() {
    if (m_driver.get() != 0) {return true;}
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
