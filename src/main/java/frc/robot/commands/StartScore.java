// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Turret;

public class StartScore extends CommandBase {
  private final Gripper m_gripper;
  private final Turret m_turret;
  /** Creates a new StartScore. */
  public StartScore(Turret turret, Carriage carriage, Gripper gripper) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    m_gripper = gripper;
    addRequirements(turret, carriage, gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gripper.toggle();
    m_turret.rotateToAngle(new TrapezoidProfile.State(180, 0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
