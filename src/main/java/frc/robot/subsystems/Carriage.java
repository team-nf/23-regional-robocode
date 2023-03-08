// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import static frc.robot.Constants.CarriageConstants.*;

public class Carriage extends SubsystemBase {
  private class Component {
    private final double kPos;
    // Neo 550.
    private final CANSparkMax m_driver;
    private final RelativeEncoder m_encoder;
    private final DoubleSolenoid m_brake;
    private final DigitalInput m_limit;

    private final PIDController pidcontroller = new PIDController(1, 0, 0);
    public Component(int motor_id, int pneumatic_id, int forward_ch, int rev_ch, int limit_ch, double encoder_position) {
      kPos = encoder_position;

      m_driver = new CANSparkMax(motor_id, MotorType.kBrushless);
      m_brake = new DoubleSolenoid(pneumatic_id, MODULE_TYPE, forward_ch, rev_ch);
      m_limit = new DigitalInput(limit_ch);
      
      m_encoder = m_driver.getEncoder(Type.kQuadrature, (int)(ENCODER_CPR));
      m_encoder.setPositionConversionFactor(DISTANCE_PER_COUNT);
      m_encoder.setPosition(encoder_position);
    }

    public void moveToAngle(int angle) {
      var voltage = pidcontroller.calculate(m_encoder.getPosition(), angle);
      m_driver.setVoltage(voltage);
    }
    
    public void brake() {m_brake.set(Value.kForward);}

    public void reset() {m_encoder.setPosition(kPos);}

    public double position() {return m_encoder.getPosition();}
    
    public boolean limit() {return m_limit.get();}

    public void move(int speed) {}

  }

  private final Component m_arm = new Component(MOTOR_ID_1, PN_ID_1, FORWARD_CHANNEL_1, REVERSE_CHANNEL_1, LIMIT_CH_1, ARM_START);
  private final Component m_wrist = new Component(MOTOR_ID_2, PN_ID_2, FORWARD_CHANNEL_2, REVERSE_CHANNEL_2, LIMIT_CH_2, WRIST_START);
  public Component wrist() {return this.m_wrist;}
  public Component arm() {return this.m_arm;}

  /** Creates a new Carriage. */
  public Carriage() {}

  // Command factories ---------------------------------------

  public CommandBase brake(Component component) {
    return this.runOnce(() -> component.brake());  
  }

  public CommandBase rotate(int angle, Component component) {
    return run(() -> component.moveToAngle(angle));
  }
  
  public CommandBase reset(Component component) {
    return runOnce(() -> component.reset());
  }

  // Conditions ----------------------------------------------

  public boolean alined() {
    if (angle() == 90.0) {return true;}
    return false;
  }

  public boolean up() {
    if (m_arm.position() > 90.0) {return true;}
    return false;
  }

  public boolean down() {
    if (m_arm.position() < 90.0) {return true;}
    return false;
  }

  public boolean armLimit() {return m_arm.limit();}
  public boolean wristLimit() {return m_wrist.limit();}

  public double angle() {
    return m_wrist.position() - m_arm.position();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
