// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.GripperConstants.*;

public class Gripper extends SubsystemBase {
  private final DoubleSolenoid m_grip = new DoubleSolenoid(PN_ID, MODULE_TYPE, FORWARD_CHANNEL, REVERSE_CHANNEL);

  // Neo 550
  private final CANSparkMax m_leftDriver = new CANSparkMax(MOTOR_ID_1, MotorType.kBrushless);
  private final CANSparkMax m_rightDriver = new CANSparkMax(MOTOR_ID_2, MotorType.kBrushless);
  private final MotorControllerGroup m_drivers;

  /** Creates a new Gripper. */
  public Gripper() {
    // this should make it so that right is inverted, follows left and i can control them with m_driver
    m_rightDriver.follow(m_leftDriver, true);
    m_drivers = new MotorControllerGroup(m_leftDriver, m_rightDriver);

    // Set solenoid
    m_grip.set(Value.kReverse);
  }

  public void toggle() {
    m_grip.toggle();
  }

  /**
   * Grip with pneumatics.
   * 
   * @return grip command that toggles solenoid.
   */
  public CommandBase grip() {
    return this.runOnce(() -> m_grip.toggle());
  }

  /**
   * Grip intake with motors.
   * 
   * @return Command to schedule at start and command to schedule after that ends.
   */
  public CommandBase intake() {
    return this.startEnd(() -> take(), () -> m_drivers.stopMotor());
  }

  /**
   * Gripper shoot with motor.
   * 
   * @return Command to schedule that sets motor to shooting speed and then ends.
   */
  public CommandBase shoot() {
    return this.startEnd(() -> start(), () -> m_drivers.stopMotor());
  }

  /**
   * Stop motors.
   * 
   * @return Command that stops motors.
   */
  public CommandBase stop() {
    return this.runOnce(() -> m_drivers.stopMotor());
  }

  /**
   * Set motor speed to shoot.
   */
  public void start() {
    m_drivers.set(SHOOTER_SPEED);
  }

  /**
   * Set motor speed to take.
   */
  public void take() {
    m_drivers.set(INTAKE_SPEED);
  }

  /**
   * Condition method
   * 
   * @return true if motors are running, false is not.
   */
  public boolean motion() {
    if (m_drivers.get() == 0) {return false;}
    return true;
  }

  // TESTING
  public CommandBase test() {return startEnd(() -> stop(), () -> stop());}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
