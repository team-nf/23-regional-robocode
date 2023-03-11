// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.TurretConstants.*;

public class Turret extends SubsystemBase {
  private double kP, kI, kD, kIz, kFF, kMinOutput, kMaxOutput, minVel, maxVel, maxAcc, maxRPM, allowedErr;
  // Neo
  private final CANSparkMax m_driver = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);

  // Encoder built-in on neo
  private final RelativeEncoder m_encoder = m_driver.getEncoder(Type.kHallSensor, (int)(ENCODER_CPR));  
  // PIDController
  private final SparkMaxPIDController pidcontroller = m_driver.getPIDController();
  // Limit switch
  private final DigitalInput m_limit = new DigitalInput(LIMIT_CH);
  private final SparkMaxLimitSwitch m_driverLimit = m_driver.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);

  /** Creates a new Turret. */
  public Turret() {
    m_encoder.setPositionConversionFactor(DISTANCE_PER_COUNT);
    this.reset();
  }

  public final void reset() {
    m_encoder.setPosition(ENCODER_START);
  }

  private final void initCoefficients() {
    // PID Coefficients
    kP = P;
    kI = I;
    kD = D;
    kIz = IZ;
    kFF = FF;
    kMinOutput = MIN_OUTPUT;
    kMaxOutput = MAX_OUTPUT;
    // Smart Motion
    minVel = MIN_VEL;
    maxVel = MAX_VEL;
    maxAcc = MAX_ACC;
    maxRPM = MAX_RPM;
    allowedErr = ALLOWED_ERR;
  }

  private final void setPID() {
    pidcontroller.setP(kP);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public CommandBase power(double speed) {
    return this.startEnd(() -> m_driver.set(speed), () -> m_driver.set(0));
  }

  public CommandBase test() {return startEnd(() -> System.out.println(m_encoder.getPosition()), () -> System.out.println(m_encoder.getPosition()));}
}
