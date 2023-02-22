// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.drivetrain.Swerve;
import frc.robot.subsystems.*;

import static frc.robot.Constants.ClawConstants.GamePiece;
import static frc.robot.subsystems.LEDs.LedMode.*;
import static frc.robot.utiliy.Calculation.deadband;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
//  private final Superstructure superstructure = new Superstructure();
//  private final Swerve swerve = new Swerve();
//  private final LEDs leds = new LEDs();

  // testing
  Arm arm = new Arm();
  Claw claw = new Claw();
  Intake intake = new Intake();
//  Spindexer spindexer = new Spindexer();

  Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

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
    // default command configurations
//    leds.setDefaultCommand(leds.setColorCommand(leds.getAlliance()));
//    swerve.setDefaultCommand(
//            swerve.dualDriveSwerveCommand(
//                    controller::getLeftX,
//                    controller::getLeftY,
//                    controller::getRightX,
//                    controller::getRightY,
//                    controller.L2(),
//                    controller.R2()));

    // pick commands
//    controller.povLeft().onTrue(superstructure.intakeCommand());
//    controller.povRight().onTrue(superstructure.intakeFromClawCommand());
//    controller.povUp().onTrue(superstructure.intakeFromShelfCommand());

    // place commands
//    controller.triangle().onTrue(superstructure.placeOnHighCommand(controller.square()).alongWith(swerve.rotateToGridCommand()));
//    controller.circle().onTrue(superstructure.placeOnMidCommand(controller.square()).alongWith(swerve.rotateToGridCommand()));
//    controller.cross().toggleOnTrue(superstructure.placeOnLowCommand(controller.square()).alongWith(swerve.rotateToGridCommand()));

    // LED control
//    controller.options().onTrue(askForGamePieceCommand(GamePiece.CONE));
//    controller.share().onTrue(askForGamePieceCommand(GamePiece.CUBE));

  controller.PS().toggleOnTrue(toggleCompressorCommand());

//     testing
//     arm

    arm.setDefaultCommand(
          arm.manualCommand(
                () -> deadband(controller.getLeftY()) / 4,
                () -> deadband(controller.getRightY())
          )
    );

    controller.triangle().toggleOnTrue(arm.floatCommand());

    controller.circle().onTrue(claw.openClawCommand());
    controller.square().onTrue(claw.closeClawCommand());

    // intake
    intake.setDefaultCommand(
          intake.manualCommand(
//                () -> deadband(controller.getLeftY()),
                ()-> 0,
                ()-> controller.getHID().getCrossButtonPressed()
          )
    );

    // spindexer
    /*
    spindexer.setDefaultCommand(
          spindexer.manualCommand(
                () -> deadband(controller.getRightY()))
    );*/
  }

//  private Command askForGamePieceCommand(GamePiece gamePiece){
//    return Commands.repeatingSequence(
//            leds.setColorCommand(gamePiece.equals(GamePiece.CONE) ? ORANGE : PURPLE),
//                    new WaitCommand(0.25),
//                    leds.setColorCommand(OFF),
//                    new WaitCommand(0.25))
//            .withTimeout(5)
//            .andThen(leds.restoreDefualtColorCommand())
//            .alongWith(superstructure.setLastRequestedGamePiece(gamePiece));
//  }

  public Command toggleCompressorCommand(){
    return new StartEndCommand(
          compressor::enableDigital,
          compressor::disable
    );
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
