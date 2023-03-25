// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.PIDCoefficients;
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
    private final String name;
    // Neo
    private final CANSparkMax m_driver;
    private final RelativeEncoder m_encoder;
    private final DoubleSolenoid m_brake;
    //private final DigitalInput m_limit;
    private final SparkMaxLimitSwitch m_driverLimit;
    
    private final SparkMaxPIDController pidcontroller;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

    /**
     * @param encoder_position encoders reset angle position
     * @param constraints [(double) maximum velocity, (double) maximum acceleration]
     * 
     * smart dashboard sorun çıkartırsa buraya 'key' isimli string parametre al smartdashboard keylerine String.format("%s/...", key) kullan.
     */
    public Component(String name, int motor_id, int forward_ch, int rev_ch, double encoder_position, PIDCoefficients coefficients) {
      kPos = encoder_position;
      this.name = name;
      
      m_driver = new CANSparkMax(motor_id, MotorType.kBrushless);
      m_brake = new DoubleSolenoid(PN_ID, MODULE_TYPE, forward_ch, rev_ch);
      //m_limit = new DigitalInput(LIMIT_CH);
      m_driverLimit = m_driver.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
      // Research SparkMaxLimitSwitch.enable
      
      
      // Encoder
      m_encoder = m_driver.getEncoder(Type.kHallSensor, (int)(ENCODER_CPR));
      m_encoder.setPositionConversionFactor(DISTANCE_PER_COUNT);
      m_encoder.setPosition(encoder_position);
      
      // PID
      // Spark Max PID Controller
      pidcontroller = m_driver.getPIDController();
      initCoefficients(coefficients);
      setRevPID();
      setSmartMotion(coefficients.PID_SLOT);

      initSmartdashboard();
    }
    
    public void moveToAngle(double angle) {
      String key = name; 
      double setPoint, processVariable;
      boolean mode = SmartDashboard.getBoolean(String.format("%s/Mode", key), false);
      if(MANUAL) {
        if(mode) {
          setPoint = SmartDashboard.getNumber(String.format("%s/Set Velocity", key), 0);
          pidcontroller.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
          processVariable = m_encoder.getVelocity();
        } else {
          setPoint = SmartDashboard.getNumber(String.format("%s/Set Position", key), 0);
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
            setPoint = SmartDashboard.getNumber(String.format("%s/Set Velocity", key), 0);
            pidcontroller.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
            processVariable = m_encoder.getVelocity();
        } else {
          setPoint = angle;
          pidcontroller.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
          processVariable = m_encoder.getPosition();
        }

          SmartDashboard.putNumber(String.format("%s/SetPoint", key), setPoint);
          SmartDashboard.putNumber(String.format("%s/Process Variable", key), processVariable);
          SmartDashboard.putNumber(String.format("%s/Output", key), m_driver.getAppliedOutput());
      }
    }

    public void move(double speed) {
      pidcontroller.setReference(speed, ControlType.kSmartVelocity);
    }
    
    public void power(double speed) {
      m_driver.set(speed);
    }
    
    public void stopMotor() {
      m_driver.stopMotor();
    }

    public void brake() {m_brake.set(Value.kForward);}

    public void coast() {m_brake.set(Value.kReverse);}    
    
    public boolean brakeState() {if (m_brake.get() == Value.kForward) {return true;} return false;}

    public void reset() {m_encoder.setPosition(kPos);}
    
    public double position() {return m_encoder.getPosition();}
    
    public boolean limit() {return m_driverLimit.isPressed();}

    //public boolean limit() {return m_limit.get();}
    
    public void move(int speed) {m_driver.set(speed);}
    
    /**Read PID coefficients from SmartDashboard, change coeffs if changed*/
    public void getUpdate(int slot) {
      String key = name; 
      double p = SmartDashboard.getNumber(String.format("%s/p", key), 0);
      double i = SmartDashboard.getNumber(String.format("%s/i", key), 0);
      double d = SmartDashboard.getNumber(String.format("%s/d", key), 0);
      double iz = SmartDashboard.getNumber(String.format("%s/iz", key), 0);
      double ff = SmartDashboard.getNumber(String.format("%s/ff", key), 0);
      double max = SmartDashboard.getNumber(String.format("%s/maxOut", key), 0);
      double min = SmartDashboard.getNumber(String.format("%s/minOut", key), 0);
      double maxV = SmartDashboard.getNumber(String.format("%s/maxVel", key), 0);
      double minV = SmartDashboard.getNumber(String.format("%s/minVel", key), 0);
      double maxA = SmartDashboard.getNumber(String.format("%s/maxAcc", key), 0);
      double allE = SmartDashboard.getNumber(String.format("%s/allowedErr", key), 0);
      
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

    private final void initCoefficients(PIDCoefficients k) {
      // PID Coefficients
      kP = k.P;
      kI = k.I;
      kD = k.D;
      kIz = k.IZ;
      kFF = k.FF;
      kMinOutput = k.MIN_OUTPUT;
      kMaxOutput = k.MAX_OUTPUT;
      // Smart Motion
      minVel = k.MIN_VEL;
      maxVel = k.MAX_VEL;
      maxAcc = k.MAX_ACC;
      maxRPM = k.MAX_RPM;
      allowedErr = k.ALLOWED_ERR;
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
      String key = name;
      // display PID coefficients on SmartDashboard
      SmartDashboard.putNumber(String.format("%s/p", key), 0);
      SmartDashboard.putNumber(String.format("%s/i", key), 0);
      SmartDashboard.putNumber(String.format("%s/d", key), 0);
      SmartDashboard.putNumber(String.format("%s/iz", key), 0);
      SmartDashboard.putNumber(String.format("%s/ff", key), 0);
      SmartDashboard.putNumber(String.format("%s/maxOut", key), 0);
      SmartDashboard.putNumber(String.format("%s/minOut", key), 0);
      
      // display Smart Motion coefficients
      SmartDashboard.putNumber(String.format("%s/maxVel", key), 0);
      SmartDashboard.putNumber(String.format("%s/minVel", key), 0);
      SmartDashboard.putNumber(String.format("%s/maxAcc", key), 0);
      SmartDashboard.putNumber(String.format("%s/allowedErr", key), allowedErr);
      SmartDashboard.putNumber(String.format("%s/Set Position", key), m_encoder.getPosition());
      SmartDashboard.putNumber(String.format("%s/Set Velocity", key), 0);

      // button to toggle between velocity and smart motion modes
      SmartDashboard.putBoolean(String.format("%s/Mode", key), true);
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
    private final String name;
    private double kP, kI, kD, kS, kG, kV, kA; 

    // Neo
    private final CANSparkMax m_driver;
    private final RelativeEncoder m_encoder;
    private final DoubleSolenoid m_brake;
    //private final DigitalInput m_limit;

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
     * @param k PID COEFFICIENTS. should be type enum {@link frc.robot.Constants.PIDCoefficients} in Constants. The enum in constants is where the values are set.
     */
    public ProfiledComponent(String name, int motor_id, int forward_ch, int rev_ch, double encoder_position, Controller controllerType, double[] constraints, frc.robot.Constants.PIDCoefficients k) {
      super(new TrapezoidProfile.Constraints(constraints[0], constraints[1]), encoder_position);
      kPos = encoder_position;
      kType = controllerType;
      this.name = name;
      
      m_driver = new CANSparkMax(motor_id, MotorType.kBrushless);
      m_brake = new DoubleSolenoid(PN_ID, MODULE_TYPE, forward_ch, rev_ch);
      //m_limit = new DigitalInput(LIMIT_CH);

      initCoefficients(k);
      feedforward = new ArmFeedforward(k.S, k.G, k.V, k.A);
      pidcontroller = new PIDController(k.P, k.I, k.D);
      
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

    public boolean brakeState() {if (m_brake.get() == Value.kForward) {return true;} return false;}
    
    public void reset() {m_encoder.setPosition(kPos);}
    
    public double position() {return m_encoder.getPosition();}
    
    public void move(int speed) {m_driver.set(speed);}

    //public boolean limit() {return m_limit.get();}

    public boolean limit() {return m_driver.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed).isPressed();}
    
    private final  void initCoefficients(PIDCoefficients k) {
      kP = k.P;
      kI = k.I;
      kD = k.D;
      kS = k.S;
      kG = k.G;
      kV = k.V;
      kA = k.A;
    }

    private final void initSmartdashboard() {
      String key = name;
      if (kType == Controller.WPILib) {
      // display PID coefficients on SmartDashboard
      SmartDashboard.putNumber(String.format("%s/p", key), 0);
      SmartDashboard.putNumber(String.format("%s/i", key), 0);
      SmartDashboard.putNumber(String.format("%s/d", key), 0);
      SmartDashboard.putNumber(String.format("%s/s", key), kS);
      SmartDashboard.putNumber(String.format("%s/g", key), kG);
      SmartDashboard.putNumber(String.format("%s/v", key), kV);
      SmartDashboard.putNumber(String.format("%s/a", key), kA);
      SmartDashboard.putNumber(String.format("%s/Set Position", key), position());
      }
      else {
        // ...
      }
    }

    /**Read PID coefficients from SmartDashboard, change coeffs if changed*/
    public void getUpdate() {
      String key = name;
      if (kType == Controller.WPILib) {
      double p = SmartDashboard.getNumber(String.format("%s/p", key), 0);
      double i = SmartDashboard.getNumber(String.format("%s/i", key), 0);
      double d = SmartDashboard.getNumber(String.format("%s/d", key), 0);
      double s = SmartDashboard.getNumber(String.format("%s/s", key), kS);
      double g = SmartDashboard.getNumber(String.format("%s/g", key), kG);
      double v = SmartDashboard.getNumber(String.format("%s/v", key), kV);
      double a = SmartDashboard.getNumber(String.format("%s/a", key), kA);

      // if PID coefficients on SmartDashboard have changed, write new values to controller
      if((p != kP)) { pidcontroller.setP(p); kP = p; }
      if((i != kI)) { pidcontroller.setI(i); kI = i; }
      if((d != kD)) { pidcontroller.setD(d); kD = d; }
      if((s != kS) || (g != kG) || (v != kV) || (a != kA)) { feedforward = new ArmFeedforward(s, g, v, a); }
      }
    }
  }
  
  private final Component m_arm = new Component("Arm", MOTOR_ID_1, FORWARD_CHANNEL_1, REVERSE_CHANNEL_1, ARM_START, ARM_PID);
  private final Component m_wrist = new Component("Wrist", MOTOR_ID_2, FORWARD_CHANNEL_2, REVERSE_CHANNEL_2, WRIST_START, WRIST_PID);
  //private final ProfiledComponent m_arm = new ProfiledComponent(MOTOR_ID_1, FORWARD_CHANNEL_1, REVERSE_CHANNEL_1,  ARM_START, ProfiledComponent.Controller.WPILib, CONSTRAINTS, frc.robot.Constants.CarriageConstants.Component.Arm);
  //private final ProfiledComponent m_wrist = new ProfiledComponent(MOTOR_ID_2, PN_ID_2, FORWARD_CHANNEL_2, REVERSE_CHANNEL_2, LIMIT_CH_2, WRIST_START, ProfiledComponent.Controller.WPILib, CONSTRAINTS);

  public Component wrist() {return this.m_wrist;}
  public Component arm() {return this.m_arm;}
  //public ProfiledComponent wrist() {return this.m_arm;}
  //public ProfiledComponent arm() {return this.m_arm;}
  
  /** Creates a new Carriage. */
  public Carriage() {}
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    update();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addBooleanProperty("Arms Pneumatic Brake", () -> m_arm.brakeState(), null);
    builder.addBooleanProperty("Wrist Pneumatic Brake", () -> m_wrist.brakeState(), null);
  }
  
  private void update() {
    m_arm.getUpdate(ARM_PID.PID_SLOT);
    m_wrist.getUpdate(WRIST_PID.PID_SLOT);
  }

  // Command factories ---------------------------------------

  public CommandBase brake(Component component) {
    return this.startEnd(() -> component.brake(), () -> component.coast());  
  }

  public CommandBase brake(ProfiledComponent component) {
    return this.startEnd(() -> component.brake(), () -> component.coast());  
  }

  public CommandBase rotate(double angle, Component component) {
    return this.runEnd(() -> {if(component.brakeState()) {component.coast();} component.moveToAngle(angle);}, () -> {component.m_driver.stopMotor(); component.brake();});
  }
  
  /**
   * @param angle Angle to set arm or wrist to.
   * @param component object of type ProfiledComponent. Use {@link rotate(int angle, Component component)} to use Spark Max Motion Profiling.
   * @return command.
   */
  public CommandBase rotate(double angle, ProfiledComponent component) {
    return this.runEnd(() -> component.setGoal(angle), () -> component.move(0));
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

  public CommandBase quickMovement(double x) {
    return this.startEnd(() -> {arm().power(-0.3 * x); wrist().power(0.3 * x);}, () -> {arm().stopMotor(); wrist().stopMotor();});
  }
  
  public CommandBase moveGripper(double x) {
    return this.startEnd(() -> {wrist().power(0.2 * x);}, () -> wrist().power(0.2 * x));
  }

  public CommandBase update(Component component) {
    return this.run(() -> component.getUpdate(0));
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
}