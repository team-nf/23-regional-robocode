// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import static frc.robot.Constants.CarriageConstants.*;

public class Carriage extends SubsystemBase {
  private final DoubleSolenoid m_brake = new DoubleSolenoid(PN_ID, MODULE_TYPE, FORWARD_CHANNEL, REVERSE_CHANNEL);
  
  // Neo 550 for moving the arm.
  private final CANSparkMax m_armDriver = new CANSparkMax(MOTOR_ID_1, MotorType.kBrushless);
  
  // Neo 550 for moving the wrist.
  private final CANSparkMax m_wristDriver = new CANSparkMax(MOTOR_ID_2, MotorType.kBrushless);
  
  // Relative yerine absolute?
  private RelativeEncoder m_armEncoder = m_armDriver.getEncoder(Type.kQuadrature, (int)(ENCODER_CPR));
  private RelativeEncoder m_wristEncoder = m_wristDriver.getEncoder(Type.kQuadrature, (int)(ENCODER_CPR));

  /** Creates a new Carriage. */
  public Carriage() {
    m_armEncoder.setPositionConversionFactor(DISTANCE_PER_COUNT);
    m_wristEncoder.setPositionConversionFactor(DISTANCE_PER_COUNT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
