// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.DriveBase;
import static frc.robot.Constants.DriveBaseConstants.*;
import static frc.robot.Constants.TestConstants.*;

/** An example command that uses an example driveBase. */
public class Drive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveBase m_driveBase;
  private final CommandXboxController m_controller;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(SPEED);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(MAX_ANGULAR_VELOCITY);

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
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed = -m_speedLimiter.calculate(m_controller.getLeftY()) * SPEED;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = -m_rotLimiter.calculate(m_controller.getLeftX()) * MAX_ANGULAR_VELOCITY;

    m_driveBase.drive(xSpeed, rot);
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
    m_driveBase.drive(TEST_CHASSIS_SPEED, TEST_CHASSIS_ANGULAR);
  }
}
