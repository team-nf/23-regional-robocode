// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.DriveBase;

/** An example command that uses an example driveBase. */
public class Drive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveBase m_driveBase;
  private final CommandXboxController m_controller;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);

  /**
   * Creates a new Drive.
   *
   * @param driveBase The driveBase used by this command.
   */
  public Drive(DriveBase driveBase, CommandXboxController controller) {
    m_driveBase = driveBase;
    m_controller = controller;
    // Use addRequirements() here to declare driveBase dependencies.
    addRequirements(driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveBase.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final var leftSpeed = -m_speedLimiter.calculate(m_controller.getLeftY());
    final var rightSpeed = -m_speedLimiter.calculate(m_controller.getRightY());
    m_driveBase.tankDrive(leftSpeed, rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void test() {
  }
}
