// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utiliy.Superstructure;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Superstructure superstructure = new Superstructure();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller controller = new CommandPS4Controller(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
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
    controller.R1().onTrue(superstructure.intakeCommand());

    superstructure.swerve.setDefaultCommand(
            superstructure.swerve.dualDriveSwerveCommand(
                    controller::getLeftX,
                    controller::getLeftY,
                    () -> -controller.getRightX(),
                    () -> controller.getRightY(),
                    controller.axisGreaterThan(1, 0.1), // TODO: find axis, and greater / lower
                    controller.axisGreaterThan(2, 0.1))); // TODO: find axis, and greater / lower

    controller.circle().onTrue(superstructure.swerve.resetGyroCommand().alongWith(superstructure.swerve.resetJoystickAngle()));

    // put game objects on the grid
    controller.triangle().onTrue(superstructure.putOnUpperCommand());
    controller.square().onTrue(superstructure.putOnMiddleCommand());
    controller.cross().onTrue(superstructure.putOnLowerCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
