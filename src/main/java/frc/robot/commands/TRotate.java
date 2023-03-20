// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// Trapzeoidal Profiling Rotation Command for Turret

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Turret;

import static frc.robot.Constants.TurretConstants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TRotate extends TrapezoidProfileCommand {
  /** Creates a new TRotate. */
  public TRotate(Turret turret, CommandXboxController controller, double input) {
    super(
        // The motion profile to be executed
        new TrapezoidProfile(
            // The motion profile constraints
            new TrapezoidProfile.Constraints(COEFF.MAX_VEL, COEFF.MAX_ACC),
            // Goal state
            new TrapezoidProfile.State(input * (DISTANCE_PER_REV * 2) + turret.position() , 0),
            // Initial state
            new TrapezoidProfile.State(turret.position(), turret.velocity())),
        state -> turret.rotateToState(state),
        turret);
      SmartDashboard.putNumber("controller input", controller.getRawAxis(0));
  }
}
