// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// COMMAND NOT IN USE! -nor coded here-
// The Gripper subsystem uses an inline command like suggested for instant commands, toggling the solenoid.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Grip extends CommandBase {
  /** Creates a new Grip. */
  public Grip() {
    // Use addRequirements() here to declare subsystem dependencies.
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
