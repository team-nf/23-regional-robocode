// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import static frc.robot.Constants.CarriageConstants.*;

public class Carriage extends SubsystemBase {
  private static class Component {
    private final double kPos;
    // Neo
    private final CANSparkMax m_driver;
    private final RelativeEncoder m_encoder;
    private final DoubleSolenoid m_brake;
    private final DigitalInput m_limit;
    private final SparkMaxLimitSwitch m_driverLimit;
    
    private final SparkMaxPIDController pidcontroller;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

    /**
     * @param encoder_position encoders reset angle position
     * @param constraints [(double) maximum velocity, (double) maximum acceleration]
     * 
     * smart dashboard sorun çıkartırsa buraya 'name' isimli string parametre al smartdashboard keylerine String.format("%s/...", name) kullan.
     */
    public Component(int motor_id, int pneumatic_id, int forward_ch, int rev_ch, int limit_ch, double encoder_position) {
      kPos = encoder_position;
      
      m_driver = new CANSparkMax(motor_id, MotorType.kBrushless);
      m_brake = new DoubleSolenoid(pneumatic_id, MODULE_TYPE, forward_ch, rev_ch);
      m_limit = new DigitalInput(limit_ch);
      m_driverLimit = m_driver.getForwardLimitSwitch(com.revrobotics.SparkMaxLimitSwitch.Type.kNormallyClosed);
      
      
      // Encoder
      m_encoder = m_driver.getEncoder(Type.kHallSensor, (int)(ENCODER_CPR));
      m_encoder.setPositionConversionFactor(DISTANCE_PER_COUNT);
      m_encoder.setPosition(encoder_position);
      
      // PID
      // Spark Max PID Controller
      pidcontroller = m_driver.getPIDController();
      initCoefficients();
      setRevPID();
      setSmartMotion(PID_SLOT);

      initSmartdashboard();
    }
    
    public void moveToAngle(double angle) {
      double setPoint, processVariable;
      boolean mode = SmartDashboard.getBoolean("Mode", false);
      if(MANUAL) {
        if(mode) {
          setPoint = SmartDashboard.getNumber("Set Velocity", 0);
          pidcontroller.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
          processVariable = m_encoder.getVelocity();
        } else {
          setPoint = SmartDashboard.getNumber("Set Position", 0);
          /**
           * As with other PID modes, Smart Motion is set by calling the
           * setReference method on an existing pid object and setting
           * the control type to kSmartMotion
           */
          pidcontroller.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
          processVariable = m_encoder.getPosition();
        }
       } else {
          if(mode) {
            setPoint = SmartDashboard.getNumber("Set Velocity", 0);
            pidcontroller.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
            processVariable = m_encoder.getVelocity();
        } else {
          setPoint = angle;
          pidcontroller.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
          processVariable = m_encoder.getPosition();
        }

          SmartDashboard.putNumber("SetPoint", setPoint);
          SmartDashboard.putNumber("Process Variable", processVariable);
          SmartDashboard.putNumber("Output", m_driver.getAppliedOutput());
      }
    }
    
    public void brake() {m_brake.set(Value.kForward);}

    public void coast() {m_brake.set(Value.kReverse);}    
    
    public void reset() {m_encoder.setPosition(kPos);}
    
    public double position() {return m_encoder.getPosition();}
    
    public boolean limit() {return m_limit.get();}
    
    public void move(int speed) {}
    
    /**Read PID coefficients from SmartDashboard, change coeffs if changed*/
    public void getUpdate(int slot) {
      double p = SmartDashboard.getNumber("P Gain", 0);
      double i = SmartDashboard.getNumber("I Gain", 0);
      double d = SmartDashboard.getNumber("D Gain", 0);
      double iz = SmartDashboard.getNumber("I Zone", 0);
      double ff = SmartDashboard.getNumber("Feed Forward", 0);
      double max = SmartDashboard.getNumber("Max Output", 0);
      double min = SmartDashboard.getNumber("Min Output", 0);
      double maxV = SmartDashboard.getNumber("Max Velocity", 0);
      double minV = SmartDashboard.getNumber("Min Velocity", 0);
      double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
      double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);
      
      // if PID coefficients on SmartDashboard have changed, write new values to controller
      if((p != kP)) { pidcontroller.setP(p); kP = p; }
      if((i != kI)) { pidcontroller.setI(i); kI = i; }
      if((d != kD)) { pidcontroller.setD(d); kD = d; }
      if((iz != kIz)) { pidcontroller.setIZone(iz); kIz = iz; }
      if((ff != kFF)) { pidcontroller.setFF(ff); kFF = ff; }
      if((max != kMaxOutput) || (min != kMinOutput)) { 
        pidcontroller.setOutputRange(min, max); 
        kMinOutput = min; kMaxOutput = max; 
      }
      if((maxV != maxVel)) { pidcontroller.setSmartMotionMaxVelocity(maxV, slot); maxVel = maxV; }
      if((minV != minVel)) { pidcontroller.setSmartMotionMinOutputVelocity(minV, slot); minVel = minV; }
      if((maxA != maxAcc)) { pidcontroller.setSmartMotionMaxAccel(maxA, slot); maxAcc = maxA; }
      if((allE != allowedErr)) { pidcontroller.setSmartMotionAllowedClosedLoopError(allE, slot); allowedErr = allE; }
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

    /**set Spark Max PID */
    private final void setRevPID() {   
      pidcontroller.setP(kP);
      pidcontroller.setI(kI);
      pidcontroller.setD(kD);
      pidcontroller.setIZone(kIz);
      pidcontroller.setFF(kFF);
      pidcontroller.setOutputRange(kMinOutput, kMaxOutput);
    }

    private final void setSmartMotion(int smartMotionSlot) {
      int slot = smartMotionSlot;
      pidcontroller.setSmartMotionMaxVelocity(maxVel, slot);
      pidcontroller.setSmartMotionMinOutputVelocity(minVel, slot);
      pidcontroller.setSmartMotionMaxAccel(maxAcc, slot);
      pidcontroller.setSmartMotionAllowedClosedLoopError(allowedErr, slot);
    }

    private final void initSmartdashboard() {
      // display PID coefficients on SmartDashboard
      SmartDashboard.putNumber("P Gain", kP);
      SmartDashboard.putNumber("I Gain", kI);
      SmartDashboard.putNumber("D Gain", kD);
      SmartDashboard.putNumber("I Zone", kIz);
      SmartDashboard.putNumber("Feed Forward", kFF);
      SmartDashboard.putNumber("Max Output", kMaxOutput);
      SmartDashboard.putNumber("Min Output", kMinOutput);
      
      // display Smart Motion coefficients
      SmartDashboard.putNumber("Max Velocity", maxVel);
      SmartDashboard.putNumber("Min Velocity", minVel);
      SmartDashboard.putNumber("Max Acceleration", maxAcc);
      SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
      SmartDashboard.putNumber("Set Position", m_encoder.getPosition());
      SmartDashboard.putNumber("Set Velocity", 0);

      // button to toggle between velocity and smart motion modes
      SmartDashboard.putBoolean("Mode", true);
    }
  }
  
  @SuppressWarnings("unused")
  private static class ProfiledComponent extends TrapezoidProfileSubsystem {
    private static enum Controller {
      WPILib,
      SparkMax
    }
    private final Controller kType; 
    private final double kPos;
    private double kP, kI, kD, kS, kG, kV, kA; 

    // Neo
    private final CANSparkMax m_driver;
    private final RelativeEncoder m_encoder;
    private final DoubleSolenoid m_brake;
    private final DigitalInput m_limit;

    // Controllers
    private final PIDController pidcontroller;
    // ArmFeedforward has no api for changing coeffficients, so i rebuild the attribute; hence, the missing 'final' keyword
    private ArmFeedforward feedforward;

    /**
     * @param encoder_position encoders reset angle position
     * @param controllerType 1 for {@link Controller.WPILib} and 2 for {@link Controller.SparkMax}.. 
     * if controllerType is set at 1 motor output will be set to a voltage calculated with a wpilib pid controller and freeforward -coefficients are set at {@link Constants} 
     * else if controllerType is set at 2 the motor output is set through the Rev API .getPIDController().setReference() on Position mode. 
     * @param constraints [(double) maximum velocity, (double) maximum acceleration]
     */
    public ProfiledComponent(int motor_id, int pneumatic_id, int forward_ch, int rev_ch, int limit_ch, double encoder_position, Controller controllerType, double[] constraints) {
      super(new TrapezoidProfile.Constraints(constraints[0], constraints[1]), encoder_position);
      kPos = encoder_position;
      kType = controllerType;
      
      m_driver = new CANSparkMax(motor_id, MotorType.kBrushless);
      m_brake = new DoubleSolenoid(pneumatic_id, MODULE_TYPE, forward_ch, rev_ch);
      m_limit = new DigitalInput(limit_ch);

      initCoefficients();
      feedforward = new ArmFeedforward(S, G, V, A);
      pidcontroller = new PIDController(P, I, D);
      
      // Encoder
      m_encoder = m_driver.getEncoder(Type.kHallSensor, (int)(ENCODER_CPR));
      m_encoder.setPositionConversionFactor(DISTANCE_PER_COUNT);
      m_encoder.setPosition(encoder_position);

      initSmartdashboard();
    }
    
    @Override
    public void useState(TrapezoidProfile.State setpoint) {
      // Calculate the feedforward from the setpoint
      // Add the feedforward to the PID output to get the motor output
      double ff = feedforward.calculate(setpoint.position, setpoint.velocity);

      if (kType == Controller.SparkMax) {
      m_driver.getPIDController().setReference(setpoint.position, ControlType.kPosition, 0, ff / 12.0);
      } else {moveToSetpoint(setpoint, ff);}
    }
    
    public void moveToSetpoint(TrapezoidProfile.State setpoint, double feedforward) {
      var voltage = pidcontroller.calculate(m_encoder.getPosition(), setpoint.position) + feedforward / 12.0;
      m_driver.setVoltage(voltage);
    }

    /**@deprecated Use outer classes rotate() factory method instead*/
    @Deprecated(forRemoval=false)
    public CommandBase moveToGoal(double angle) {
      return this.runOnce(() -> setGoal(angle));
    }
    
    public void brake() {m_brake.set(Value.kForward);}
   
    public void coast() {m_brake.set(Value.kReverse);} 
    
    public void reset() {m_encoder.setPosition(kPos);}
    
    public double position() {return m_encoder.getPosition();}
    
    public void move(int speed) {}
    
    public boolean limit() {return m_limit.get();}
    
    private final  void initCoefficients() {
      kP = P;
      kI = I;
      kD = D;
      kS = S;
      kG = G;
      kV = V;
      kA = A;
    }

    private final void initSmartdashboard() {
      if (kType == Controller.WPILib) {
      // display PID coefficients on SmartDashboard
      SmartDashboard.putNumber("P Gain", kP);
      SmartDashboard.putNumber("I Gain", kI);
      SmartDashboard.putNumber("D Gain", kD);
      SmartDashboard.putNumber("S Gain", kS);
      SmartDashboard.putNumber("G Gain", kG);
      SmartDashboard.putNumber("V Gain", kV);
      SmartDashboard.putNumber("A Gain", kA);
      SmartDashboard.putNumber("Set Position", position());
      }
      else {
        // ...
      }
    }

    /**Read PID coefficients from SmartDashboard, change coeffs if changed*/
    public void getUpdate() {
      if (kType == Controller.WPILib) {
      double p = SmartDashboard.getNumber("P Gain", 0);
      double i = SmartDashboard.getNumber("I Gain", 0);
      double d = SmartDashboard.getNumber("D Gain", 0);

      double s = SmartDashboard.getNumber("S Gain", kS);
      double g = SmartDashboard.getNumber("G Gain", kG);
      double v = SmartDashboard.getNumber("V Gain", kV);
      double a = SmartDashboard.getNumber("A Gain", kA);

      // if PID coefficients on SmartDashboard have changed, write new values to controller
      if((p != kP)) { pidcontroller.setP(p); kP = p; }
      if((i != kI)) { pidcontroller.setI(i); kI = i; }
      if((d != kD)) { pidcontroller.setD(d); kD = d; }
      if((s != kS) || (g != kG) || (v != kV) || (a != kA)) { feedforward = new ArmFeedforward(s, g, v, a); }
      }
    }
  }
  
  private final Component m_arm = new Component(MOTOR_ID_1, PN_ID_1, FORWARD_CHANNEL_1, REVERSE_CHANNEL_1, LIMIT_CH_1, ARM_START);
  private final Component m_wrist = new Component(MOTOR_ID_2, PN_ID_2, FORWARD_CHANNEL_2, REVERSE_CHANNEL_2, LIMIT_CH_2, WRIST_START);
  //private final ProfiledComponent m_arm = new ProfiledComponent(MOTOR_ID_1, PN_ID_1, FORWARD_CHANNEL_1, REVERSE_CHANNEL_1, LIMIT_CH_1, ARM_START, ProfiledComponent.Controller.WPILib, CONSTRAINTS);
  //private final ProfiledComponent m_wrist = new ProfiledComponent(MOTOR_ID_2, PN_ID_2, FORWARD_CHANNEL_2, REVERSE_CHANNEL_2, LIMIT_CH_2, WRIST_START, ProfiledComponent.Controller.WPILib, CONSTRAINTS);

  public Component wrist() {return this.m_wrist;}
  public Component arm() {return this.m_arm;}
  //public ProfiledComponent wrist() {return this.m_arm;}
  //public ProfiledComponent arm() {return this.m_arm;}

  /** Creates a new Carriage. */
  public Carriage() {}

  // Command factories ---------------------------------------

  public CommandBase brake(Component component) {
    return this.startEnd(() -> component.brake(), () -> component.coast());  
  }

  public CommandBase brake(ProfiledComponent component) {
    return this.startEnd(() -> component.brake(), () -> component.coast());  
  }

  public CommandBase rotate(double angle, Component component) {
    return this.run(() -> component.moveToAngle(angle));
  }
  
  /**
   * @param angle Angle to set arm or wrist to.
   * @param component object of type ProfiledComponent. Use {@link rotate(int angle, Component component)} to use Spark Max Motion Profiling.
   * @return command.
   */
  public CommandBase rotate(double angle, ProfiledComponent component) {
    return this.run(() -> component.setGoal(angle));
  }

  /**
   * @deprecated CONTROLLER TYPE IS NOT CHANGEABLE AT THE MOMENT. Will look into it later.
   * for time being, use {@link rotate(double angle, ProfiledComponent component)}
   * 
   * @param angle Angle to set arm or wrist to.
   * @param component object of type ProfiledComponent. Use {@link rotate(int angle, Component component)} to use Spark Max Motion Profiling.
   * @param controllerType 1 for {@link Controller.WPILib} and 2 for {@link Controller.SparkMax}.. 
   * if controllerType is set at 1 motor output will be set to a voltage calculated with a wpilib pid controller and freeforward -coefficients are set at {@link Constants} 
   * else if controllerType is set at 2 the motor output is set through the Rev API .getPIDController().setReference() on Position mode. 
   * @return command.
   */
  @Deprecated(forRemoval=false)
  public CommandBase rotate(double angle, ProfiledComponent component, int controllerType) {
    return this.run(() -> component.setGoal(angle));
  }

  public CommandBase update(Component component) {
    return this.run(() -> component.getUpdate(PID_SLOT));
  }

  public CommandBase update(ProfiledComponent component) {
    return this.run(() -> component.getUpdate());
  }
  
  public CommandBase reset(Component component) {
    return this.runOnce(() -> component.reset());
  }

  public CommandBase reset(ProfiledComponent component) {
    return this.runOnce(() -> component.reset());
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

  // TEST
  public CommandBase test() {return startEnd(() -> System.out.println(m_arm.limit()), () -> System.out.println(m_wrist.limit()));}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}