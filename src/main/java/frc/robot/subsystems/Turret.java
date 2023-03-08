// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.TurretConstants.*;

public class Turret extends SubsystemBase {
  // Neo 550
  private final CANSparkMax m_driver = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);

  // Encoder built-in on neo550
  private final RelativeEncoder m_encoder = m_driver.getEncoder(Type.kQuadrature, (int)(ENCODER_CPR));  
  // Limit switch
  private final DigitalInput m_limit = new DigitalInput(LIMIT_CH);

  /** Creates a new Turret. */
  public Turret() {
    m_encoder.setPositionConversionFactor(DISTANCE_PER_COUNT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
