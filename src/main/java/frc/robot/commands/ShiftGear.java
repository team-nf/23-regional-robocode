// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// COMMAND NOT IN USE!
// Code is still here as an "in case" option.
// The drivebase.shiftGear() method used in this code is commented out in the subsystem and replaced.
// The rewritten drivebase.shiftGear method now uses an inline command like suggested below.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveBase;
import frc.wpilibj9029.Shifter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShiftGear extends InstantCommand {
  private final DriveBase m_driveBase;
  //private int state;
  //private final Shifter m_shifter; 
  //private int state;

  public ShiftGear(DriveBase drivebase) {
    // Use addRequirements() here to declare subsystem dependencies.
    //m_shifter = shifter;
    m_driveBase = drivebase;

    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {m_driveBase.shiftGear();}
}
