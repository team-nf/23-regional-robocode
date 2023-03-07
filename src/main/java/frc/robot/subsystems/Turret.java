// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.TurretConstants.*;

public class Turret extends SubsystemBase {
  // Neo 550
  private final CANSparkMax m_driver = new CANSparkMax(0, MotorType.kBrushless);

  // Encoder built-in on neo550
  private final RelativeEncoder m_encoder = m_driver.getEncoder(Type.kQuadrature, (int)Math.round(DISTANCE_PER_REV * 100));
  // The encoder is going to count 419 per revolution. This is because the revrobotics api only accepts integer values
  // (as far as i understand) and the turrent spins 4.1999 degrees{DISTANCE_PER_REV} every revolution.
  // I'm sure we would want it to not turn after a certain amount and rounding it from 4.2 to 4 is to big of a round, as it results in a 1 degree loss every 5 revolutions
  // Continue coding either counting 36000 as 360 degrees or by dividing encoder values by 100
  
  //
  

  /** Creates a new Turret. */
  public Turret() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
