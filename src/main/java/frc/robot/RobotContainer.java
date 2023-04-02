// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExtensionPositionCommand;
import frc.robot.commands.IntakeControllerCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NavXGyro;
import frc.robot.commands.ArmControllerCommand;
import frc.robot.commands.ArmPositionCommand;
import frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public static NavXGyro _gyro = NavXGyro.getInstance(); // This must be called before Drive as it is used by the Drive
  public static Drive _drive = Drive.getInstance(_gyro);
  public static Intake _intake = Intake.getInstance();
  public static Arm _arm = Arm.getInstance();
  
  public final CommandJoystick leftStick = new CommandJoystick(OperatorConstants.LeftStick);
  public final CommandJoystick rightStick = new CommandJoystick(OperatorConstants.RightStick);
  public final CommandXboxController opController = new CommandXboxController(OperatorConstants.OpController);

  // Setup Sendable chooser for picking autonomous program in SmartDashboard
  private SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    CommandScheduler.getInstance()
        .setDefaultCommand(_drive, new DriveCommand(_drive, leftStick, rightStick, _gyro));

    CommandScheduler.getInstance()
        .setDefaultCommand(_arm, new ArmControllerCommand(_arm, opController));

    CommandScheduler.getInstance()
    .setDefaultCommand(_intake, new IntakeControllerCommand(_intake, opController));

    // Configure Autonomous Options
    autonomousOptions();

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Reset NavX
    leftStick.button(7).onTrue(new InstantCommand(() -> _gyro.zeroNavHeading(), _gyro));

    rightStick.button(3).toggleOnTrue(new ConditionalCommand(
        new InstantCommand(() -> _drive.setDriveModeBrake()),
        new InstantCommand(() -> _drive.setDriveModeCoast()),
        () -> _drive.toggleMode()
      ));

    opController.a().whileTrue(new ArmPositionCommand(_arm, ArmConstants.armShoulderPosition, ArmConstants.armExtensionPosition, 3));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Get the selected Auto in smartDashboard
    return m_chooser.getSelected();
  }

  /**
   * Use this to set Autonomous options for selection in Smart Dashboard
   */
  private void autonomousOptions() {
    // Example adding Autonomous option to chooser
    m_chooser.addOption("Do Nothing", Autos.doNothing());
    m_chooser.addOption("Cable Straight", Autos.cableDriveStraight(_drive, _arm, _intake));
    m_chooser.addOption("Center Ramp", Autos.centerRamp(_drive, _gyro, _arm, _intake));
    //m_chooser.addOption("Barrier Straight", Autos.barrierDriveStraight(_drive));
    m_chooser.addOption("Far Barrier Cube Score", Autos.farBarrierCubeScoreLow(_drive, _intake, _arm));
    m_chooser.addOption("Barrier Cone", Autos.barrierCone(_drive, _gyro, _intake, _arm));
    m_chooser.addOption("Blue Barrier Cone Ramp", Autos.blueBarrierConeRamp(_drive, _gyro, _intake, _arm));
    
    // Put the chooser on the dashboard
    SmartDashboard.putData(m_chooser);
  }
}
