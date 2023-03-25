// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveBaseConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.AutonomousConstants.Auto;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.commands.AutoPickup;
import frc.robot.commands.Autos;
import frc.robot.commands.Drive;
import frc.robot.commands.PRotate;
import frc.robot.commands.Score;
import frc.robot.commands.StartScore;
import frc.robot.commands.TRotate;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Gripper;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
  NetworkTable optable = ntInst.getTable("ShuffleBoard/CompetitionDashboard");
  IntegerEntry ledEntry = ntInst.getTable("datatable").getIntegerTopic("LED").getEntry(0);

  double[] apriltag = {0.0, 0.0, 0.0, 0.0, 0.0};
  double[] reflectivetape = {0.0, 0.0, 0.0, 0.0, 0.0};
  DoubleArrayEntry aprilEntry = ntInst.getTable("datatable").getDoubleArrayTopic("AprilTag").getEntry(apriltag);
  DoubleArrayEntry tapeEntry = ntInst.getTable("database").getDoubleArrayTopic("ReflectiveTape").getEntry(reflectivetape);

  // The robot's subsystems are defined here...
  private final DriveBase m_driveBase = new DriveBase();
  private final RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
    m_driveBase::getPose2d,
    m_driveBase::resetPose,
    new RamseteController(2, 0.7),
    m_driveBase.kinematics(),
    (Double leftOutput, Double rightOutput) -> {m_driveBase.setSpeeds(new DifferentialDriveWheelSpeeds(leftOutput, rightOutput));},
    AutonomousConstants.eventMap,
    m_driveBase
  );
  private final Turret m_turret = new Turret();
  private final Lift m_lift = new Lift();
  private final Carriage m_carriage = new Carriage();
  private final Gripper m_gripper = new Gripper();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
  new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  private final CommandXboxController m_operatorController =
  new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);
  
  // The robot's commands are defined here
  private final Drive m_driveCommand = new Drive(m_driveBase, m_driverController);

  // Set the Event Mappings of the robot.
  {AutonomousConstants.eventMap.put("pickup", new AutoPickup());
  AutonomousConstants.eventMap.put("start", new StartScore(m_turret, m_carriage, m_gripper));
  AutonomousConstants.eventMap.put("shifter", m_driveBase.shiftGear());
  AutonomousConstants.eventMap.put("point", new Score());}

  // Smart Dashboard
  private final SendableChooser<Command> m_autoSelector = new SendableChooser<>();
  {
    //*0 - simple center, 1 - charger station center, 2 - simple top */
    m_autoSelector.setDefaultOption("Set From Code", getAutonomousCommand());
    m_autoSelector.addOption("Top Simple", getAutonomousCommand(2));
    m_autoSelector.addOption("Top Charge Station", getAutonomousCommand(3));
    m_autoSelector.addOption("Center Simple", getAutonomousCommand(0));
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_driveBase.setDefaultCommand(m_driveCommand);
    OperatorConstants.DRIVER_DASHBOARD.add("Autonomous Route", m_autoSelector).withWidget(BuiltInWidgets.kComboBoxChooser);
    ledEntry.set(1);
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
    //new Trigger(subsystem::condition).onTrue(command());

    // Drive Base
    // Schedule command to shift gear back to default when condition changes to `true` -when not moving and shifter is not default-
    //new Trigger(m_driveBase::motion).negate().and(m_driveBase::shifterCondition).onFalse(m_driveBase.shiftGear());
    
    // Turret
    new Trigger(m_turret::limit).onTrue(m_turret.brake());

    // Lift
    // Schedule brake command when boths conditions are true -lift is not in motion and switch is hit-
    new Trigger(m_lift::panicCondition).and(m_lift::motion).onTrue(m_lift.brake());

    // Carriage
    new Trigger(m_carriage::armLimit).onTrue(m_carriage.reset(m_carriage.arm()));
    new Trigger(m_carriage::wristLimit).onTrue(m_carriage.reset(m_carriage.wrist()));

    // Drive Base
    // Schedule command to shift gears when the Xbox controller's B button is pressed, cancelling on release.
    m_driverController.b().whileTrue(m_driveBase.shiftGear());
    // Switch Driving mode
    m_driverController.a().onTrue(m_driveBase.toggleMode());
    // Brake
    m_driverController.x().whileTrue(m_driveBase.brake());
    // LEDs
    m_driverController.leftBumper().whileTrue(Commands.startEnd(() -> ledEntry.set(2), () -> ledEntry.set(1)));
    m_driverController.rightBumper().whileTrue(Commands.startEnd(() -> ledEntry.set(3), () -> ledEntry.set(1)));
    m_driverController.leftBumper().and(m_driverController.rightBumper()).debounce(.3).onTrue(Commands.runOnce(() -> ledEntry.set(0)));

    // Turret
    m_operatorController.b().whileTrue(m_turret.power(0));
    m_operatorController.axisGreaterThan(0, 0.3).whileTrue(new TRotate(m_turret, m_operatorController, 1).andThen(m_turret.brake()));
    m_operatorController.axisLessThan(0, -0.3).whileTrue(new TRotate(m_turret, m_operatorController, -1).andThen(m_turret.brake()));
    m_operatorController.axisGreaterThan(1, 0.5).whileTrue(m_turret.power(-0.2));
    m_operatorController.axisLessThan(1, -0.5).whileTrue(m_turret.power(0.2));

    // Lift
    m_operatorController.leftTrigger().whileTrue(m_lift.lift());
    m_operatorController.rightTrigger().whileTrue(m_lift.lift());

    // Carriage
    //m_operatorController.x().onTrue(m_carriage.rotate(90, m_carriage.arm()));
    //m_operatorController.leftStick().whileTrue(m_carriage.rotate(160, m_carriage.arm()));
    m_operatorController.axisGreaterThan(2, 0.2).whileTrue(m_carriage.quickMovement(-1));
    m_operatorController.axisLessThan(2, -0.2).whileTrue(m_carriage.quickMovement(1));
    m_operatorController.a().whileTrue(m_carriage.moveGripper(-1));
    m_operatorController.y().whileTrue(m_carriage.moveGripper(1));

    // Gripper
    m_operatorController.leftBumper().toggleOnTrue(m_gripper.grip());
    m_operatorController.x().onTrue(m_gripper.shoot().withTimeout(0.5));
    m_operatorController.rightBumper().whileTrue(m_gripper.intake());

  } 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    switch (AutonomousConstants.AUTO) {
      case SIMPLE:
      switch (AutonomousConstants.AUTO.pos()) {
        case TOP: return autoBuilder.fullAuto(Autos.simpleTop);
        case CENTER: return autoBuilder.fullAuto(Autos.simpleCenter);
        case BOTTOM:
      }
      case CHARGED:
        switch (AutonomousConstants.AUTO.pos()) {
          case TOP: return autoBuilder.fullAuto(Autos.chargedTop);
          case CENTER: return autoBuilder.fullAuto(Autos.chargedCenter);
          case BOTTOM:
        }
      default:
        return getAutonomousCommand();
    }
  }

  private Command getAutonomousCommand(int pathNumber) {
    switch (pathNumber) {
      case 0:
        return autoBuilder.fullAuto(Autos.simpleCenter);
      case 1:
        return autoBuilder.fullAuto(Autos.chargedCenter);
      case 2:
        return autoBuilder.fullAuto(Autos.simpleTop);
      case 3:
        return autoBuilder.fullAuto(Autos.chargedTop);
      case 4:
        return null;
      case 5:
        return null;
      case 6:
        return null;
      default:
        return getAutonomousCommand();
    }
  }
}
