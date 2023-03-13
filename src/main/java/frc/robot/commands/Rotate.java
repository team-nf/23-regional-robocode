// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class Rotate extends CommandBase {
  /** Creates a new RotateTurret. */
  private final Turret m_turret;
  public Rotate(Turret turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SmartDashboard
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_turret.limit()) {
      if (m_turret.position() < 0) {

      }
      else if (m_turret.position() > 0) {

      }
    }
    else {

    }
  }

  /**
   * @Override
   * public void execute() {
   *  if (m_turret.topLimit()) {
   * 
   *  }
   *  else if (m_turret.bottomLimit()) {
   * 
   *  }
   *  else {
   * 
   *  }
   * }
   */

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
