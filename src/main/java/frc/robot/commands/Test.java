// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Lift;;

public class Test extends CommandBase {
  private final DriveBase m_driveBase;
  private final Turret m_turret;
  private final Lift m_lift;
  private final Carriage m_carriage;
  private final Gripper m_gripper;

  /** Creates a new Test. */
  public Test(DriveBase drivebase, Turret turret, Lift lift, Carriage carriage, Gripper gripper) {
    m_driveBase = drivebase;
    m_turret = turret;
    m_lift = lift;
    m_carriage = carriage;
    m_gripper = gripper;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveBase, m_turret, m_lift, m_carriage, m_gripper);
  }

  public CommandBase factory() {
    return Commands.sequence(m_driveBase.test(), m_turret.test(), m_lift.test(), m_carriage.test(), m_gripper.test());
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

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
