// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static frc.robot.Constants.TurretConstants.*;

import javax.xml.namespace.QName;

public class Turret extends SubsystemBase {
  private double kP, kI, kD, kIz, kFF, kS, kG, kV, kA, kMinOutput, kMaxOutput, minVel, maxVel, maxAcc, maxRPM, allowedErr;
  // Neo
  private final CANSparkMax m_driver = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);

  // Encoder built-in on neo
  private final RelativeEncoder m_encoder = m_driver.getEncoder(Type.kHallSensor, (int)(ENCODER_CPR));  
  // PIDController
  private final SparkMaxPIDController pidcontroller = m_driver.getPIDController();
  private final PIDController secondarypid = new PIDController(COEFF.P, COEFF.I, COEFF.D);

  // Limit switch
  private final DigitalInput m_limit = new DigitalInput(LIMIT_CH);

  /** Creates a new Turret. */
  public Turret() {
    m_driver.setInverted(true); 
    m_encoder.setPositionConversionFactor(DISTANCE_PER_REV);
    m_encoder.setVelocityConversionFactor(DISTANCE_PER_REV);
    this.reset();
    initCoefficients();
    setPID();
    initSmartdashboard();
  }
  
  public double position() {
    return m_driver.getEncoder().getPosition();
  }
  
  public double velocity() {
    return m_driver.getEncoder().getVelocity();
  }

  public final void reset() {
    m_encoder.setPosition(ENCODER_START);
  }
  
  private final void initSmartdashboard() {
    SmartDashboard.putNumber("Turret/p", kP);
    SmartDashboard.putNumber("Turret/i", kI);
    SmartDashboard.putNumber("Turret/d", kD);
    SmartDashboard.putNumber("Turret/iz", kIz);
    SmartDashboard.putNumber("Turret/ff", kFF);
    SmartDashboard.putNumber("Turret/s", kS);
    SmartDashboard.putNumber("Turret/v", kV);
    SmartDashboard.putNumber("Turret/g", kG);
    SmartDashboard.putNumber("Turret/a", kA);
    SmartDashboard.putNumber("Turret/maxOut", kMaxOutput);
    SmartDashboard.putNumber("Turret/minOut", kMinOutput);
    SmartDashboard.putNumber("Turret/maxVel", maxVel);
    SmartDashboard.putNumber("Turret/minVel", minVel);
    SmartDashboard.putNumber("Turret/maxAcc", maxAcc);
    SmartDashboard.putNumber("Turret/allowedErr", allowedErr);
    SmartDashboard.putNumber("Turret/position", m_encoder.getPosition());
  } 
  
  private final void initCoefficients() {
    // PID Coefficients
    kP = COEFF.P;
    kI = COEFF.I;
    kD = COEFF.D;
    kS = COEFF.S;
    kG = COEFF.G;
    kV = COEFF.V;
    kA = COEFF.A;
    kIz = COEFF.IZ;
    kFF = COEFF.FF;
    kMinOutput = COEFF.MIN_OUTPUT;
    kMaxOutput = COEFF.MAX_OUTPUT;
    // Smart Motion
    minVel = COEFF.MIN_VEL;
    maxVel = COEFF.MAX_VEL;
    maxAcc = COEFF.MAX_ACC;
    maxRPM = COEFF.MAX_RPM;
    allowedErr = COEFF.ALLOWED_ERR;
  }

  private final void setPID() {
    pidcontroller.setP(0, 0);
    pidcontroller.setI(0, 0);
    pidcontroller.setD(0, 0);
    pidcontroller.setP(kP, COEFF.PID_SLOT);
    pidcontroller.setI(kI, COEFF.PID_SLOT);
    pidcontroller.setD(kD, COEFF.PID_SLOT);
    secondarypid.setPID(kP, kI, kD);
    pidcontroller.setIZone(kIz, COEFF.PID_SLOT);
    pidcontroller.setFF(kFF, COEFF.PID_SLOT);
    pidcontroller.setOutputRange(kMinOutput, kMaxOutput, COEFF.PID_SLOT);
    pidcontroller.setSmartMotionMaxVelocity(maxVel, COEFF.PID_SLOT);
    pidcontroller.setSmartMotionMinOutputVelocity(minVel, COEFF.PID_SLOT);
    pidcontroller.setSmartMotionMaxAccel(maxAcc, COEFF.PID_SLOT);
    pidcontroller.setSmartMotionAllowedClosedLoopError(allowedErr, COEFF.PID_SLOT);
  }

  /**Read PID coefficients from SmartDashboard, change coeffs if changed*/
  private void update() {
    double p = SmartDashboard.getNumber("Turret/p", 0);
    double i = SmartDashboard.getNumber("Turret/i", 0);
    double d = SmartDashboard.getNumber("Turret/d", 0);
    double iz = SmartDashboard.getNumber("Turret/iz", 0);
    double ff = SmartDashboard.getNumber("Turret/ff", 0);
    double s = SmartDashboard.getNumber("Turret/s", 0);
    double g = SmartDashboard.getNumber("Turret/g", 0);
    double v = SmartDashboard.getNumber("Turret/v", 0);
    double a = SmartDashboard.getNumber("Turret/a", 0);
    double max = SmartDashboard.getNumber("Turret/maxOut", 0);
    double min = SmartDashboard.getNumber("Turret/minOut", 0);
    double maxV = SmartDashboard.getNumber("Turret/maxVel", 0);
    double minV = SmartDashboard.getNumber("Turret/minVel", 0);
    double maxA = SmartDashboard.getNumber("Turret/maxAcc", 0);
    double allE = SmartDashboard.getNumber("Turret/allowedErr", 0);
    
    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { pidcontroller.setP(p); secondarypid.setP(p); kP = p; }
    if((i != kI)) { pidcontroller.setI(i); secondarypid.setI(i); kI = i; }
    if((d != kD)) { pidcontroller.setD(d); secondarypid.setD(d); kD = d; }
    if((iz != kIz)) { pidcontroller.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { pidcontroller.setFF(ff); kFF = ff; }
    if((s != kS)) { kS = s; }
    if((v != kV)) { kV = v; }
    if((g != kG)) { kG = g; } 
    if((a != kA)) { kA = a; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      pidcontroller.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxV != maxVel)) { pidcontroller.setSmartMotionMaxVelocity(maxV, COEFF.PID_SLOT); maxVel = maxV; }
    if((minV != minVel)) { pidcontroller.setSmartMotionMinOutputVelocity(minV, COEFF.PID_SLOT); minVel = minV; }
    if((maxA != maxAcc)) { pidcontroller.setSmartMotionMaxAccel(maxA, COEFF.PID_SLOT); maxAcc = maxA; }
    if((allE != allowedErr)) { pidcontroller.setSmartMotionAllowedClosedLoopError(allE, COEFF.PID_SLOT); allowedErr = allE; }
     

    SmartDashboard.putNumber("Turret/position", position());
  } 
  
  private void rotateFromDashboard() {
    double setPoint, processVariable;
    setPoint = SmartDashboard.getNumber("Turret/Set Position", 0);
    pidcontroller.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
    processVariable = m_encoder.getPosition();
    
    SmartDashboard.putNumber("Turret/SetPoint", setPoint);
    SmartDashboard.putNumber("Turret/Process Variable", processVariable);
    SmartDashboard.putNumber("Turret/Output", m_driver.getAppliedOutput());
  }

  public void rotateByInput(double input) {

  } 

  public void rotateToAngle(TrapezoidProfile.State state) {
    if (limit()) {
      rotateToAngle(new TrapezoidProfile.State(-position(), 0));
      return;
    }
    double ff = new SimpleMotorFeedforward(kS, kV, kA).calculate(state.velocity);
    pidcontroller.setFF(ff, COEFF.PID_SLOT);
    pidcontroller.setReference(state.position, ControlType.kPosition, COEFF.PID_SLOT);
    //double pid = secondarypid.calculate(position(), angle);
    //m_driver.setVoltage(pid + ff);
  }

   /**
   * Attempts to follow the given drive states using trapezoidal profile.
   *
   * @param state The turret state.
   */
  public void rotateToState(TrapezoidProfile.State state) {
    if (limit()) {
      rotateToAngle(new TrapezoidProfile.State(0, 0));
      return;
    }
    double ff = new SimpleMotorFeedforward(kS, kV).calculate(state.velocity);
    pidcontroller.setReference(state.position, ControlType.kPosition, 0, ff);
  }
   
  public void turn(double speed) {
    m_driver.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    update();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addBooleanProperty("Turret/Limit Switch",  m_limit::get, null);;
  }

  public CommandBase rotate(double angle) {
    return this.runOnce(() -> pidcontroller.setReference(angle, ControlType.kPosition, COEFF.PID_SLOT));
    //return new TrapezoidProfileCommand(
    //  new TrapezoidProfile(new TrapezoidProfile.Constraints(COEFF.MAX_VEL, COEFF.MAX_ACC),
    //  new TrapezoidProfile.State(angle, 0),
    //  new TrapezoidProfile.State(position(), velocity())), 
    //  state -> this.rotateToAngle(state), 
    //  this);
  }

  public void simpleRotate(double angle) {
    pidcontroller.setReference(angle, ControlType.kPosition, COEFF.PID_SLOT);
  }

  public CommandBase power(double speed) {
    return this.startEnd(() -> m_driver.set(speed), () -> m_driver.set(0));
  }
 
  public CommandBase brake() {
    return this.runOnce(() -> m_driver.stopMotor());
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

  /**
   * Condition method.
   * Panic if limit switch is hit.
   * Used for the brake.
   * 
   * @return True for panic, false for OK.
   */
  public boolean limit() {
    if(m_limit.get()) {return true;}
    return false;
  }

  public CommandBase test() {return startEnd(() -> System.out.println(m_encoder.getPosition()), () -> System.out.println(m_encoder.getPosition()));}
}
