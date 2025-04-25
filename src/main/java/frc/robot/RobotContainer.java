// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Example;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Example m_example = new Example();
  private final Drive m_drive = new Drive();
  private final Intake m_intake = new Intake();
  private final Climb m_climb = new Climb();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_controller =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));
    m_drive.setDefaultCommand(new RunCommand(
                                  () -> {m_drive.driveOpenLoop(m_controller.getLeftY(), m_controller.getRightX());},
                                  m_drive));

    // Intake Algae
    new Trigger(m_controller.b()
                            .onTrue(new InstantCommand(
                                        () -> m_intake.requestState(IntakeStates.IntakeAlgae), 
                                        m_intake)));

    // Reset Intake
    new Trigger(m_controller.a()
                            .onTrue(new InstantCommand(
                                        () -> m_intake.requestState(IntakeStates.Idle), 
                                        m_intake)));

    // Outtake Algae
    new Trigger(m_controller.leftTrigger(0.1)
                            .onTrue(new InstantCommand(
                                        () -> m_intake.requestState(IntakeStates.OuttakeAlgae),
                                        m_intake)))
                            .onFalse(new InstantCommand(
                                        () -> m_intake.requestState(IntakeStates.Idle), 
                                        m_intake));
    
    // Outtake Coral
    new Trigger(m_controller.rightTrigger(0.1)
                            .onTrue(new InstantCommand(
                                        () -> m_intake.requestState(IntakeStates.OuttakeCoral), 
                                        m_intake)))
                            .onFalse(new InstantCommand(
                                          () -> m_intake.requestState(IntakeStates.Idle), 
                                          m_intake));

    // Toggle Pyramid
    new Trigger(m_controller.rightBumper()
                            .onTrue(new InstantCommand(
                                        () -> m_intake.togglePyramidMode(), 
                                        m_intake)));
    

    // Climb
    new Trigger(m_controller.y()
                            .onTrue(new InstantCommand(
                            () -> m_climb.climb(),
                            m_climb)));

    new Trigger(m_controller.x()
                            .onTrue(new InstantCommand(
                            () -> m_climb.unclimb(),
                            m_climb)));

    // Original comments - TODO(landon): remove?                                          
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_controller.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_example);
  }
}
