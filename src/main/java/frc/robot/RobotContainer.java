// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.Autos;
import frc.robot.commands.Drive;

import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Gripper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems are defined here...
  private final DriveBase m_driveBase = new DriveBase();
  private final Turret m_turret = new Turret();
  private final Lift m_lift = new Lift();
  private final Carriage m_carriage = new Carriage();
  private final Gripper m_gripper = new Gripper();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
  new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  
  // The robot's commands are defined here
  private final Drive m_driveCommand = new Drive(m_driveBase, m_driverController);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_driveBase.setDefaultCommand(m_driveCommand);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule command when condition changes to `true`
    //new Trigger(m_driveBase::condition).onTrue(new command());
    new Trigger(m_driveBase::motion).negate().and(m_driveBase::shifterCondition).onTrue(m_driveBase.shiftGear());
    new Trigger(m_lift::panicCondition).and(m_lift::motion).onTrue(m_lift.brake());

    // Schedule command when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_driveBase.shiftGear());
  } 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_driveBase);
  }
}
