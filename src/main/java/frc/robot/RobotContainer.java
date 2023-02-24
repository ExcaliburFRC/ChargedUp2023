// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
import frc.robot.utiliy.ToggleCommand;

import static frc.robot.utiliy.Calculation.deadband;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Superstructure superstructure = new Superstructure();
//  private final Swerve swerve = new Swerve();
  //  private final LEDs leds = new LEDs();
  Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

  // testing
//  Arm arm = new Arm();
//  Claw claw = new Claw();
  Intake intake = new Intake();

  public final CommandPS4Controller driveJoystick = new CommandPS4Controller(0);
  public final CommandJoystick armJoystick = new CommandJoystick(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    SmartDashboard.putData("arm subsystem", superstructure.arm);
//    SmartDashboard.putData("claw", claw);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * edu.wpi.first.wpilibj2.command.button.CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} driveJoysticks or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // default command configurations
//    leds.setDefaultCommand(leds.setColorCommand(leds.getAlliance()));
//    swerve.setDefaultCommand(
//          swerve.driveSwerveCommand(
//                driveJoystick::getLeftX,
//                driveJoystick::getLeftY,
//                driveJoystick::getRightX,
//                driveJoystick.L2().negate()));

    // intake command
    driveJoystick.square().onTrue(superstructure.intakeFromShelfCommand(driveJoystick.R1()));

    // place commands
    driveJoystick.triangle().onTrue(superstructure.placeOnHighCommand(driveJoystick.square(), driveJoystick.L1()));
    driveJoystick.circle().onTrue(superstructure.placeOnMidCommand(driveJoystick.square(), driveJoystick.L1()));
    driveJoystick.cross().onTrue(superstructure.placeOnLowCommand(driveJoystick.square(), driveJoystick.L1()));

    // LED control
//    driveJoystick.options().onTrue(askForGamePieceCommand(GamePiece.CONE));
//    driveJoystick.share().onTrue(askForGamePieceCommand(GamePiece.CUBE));

    // other
    driveJoystick.PS().toggleOnTrue(toggleCompressorCommand());
//    driveJoystick.R1().onTrue(swerve.resetGyroCommand());

//     --- testing ---
    compressor.disable();

    driveJoystick.povLeft().toggleOnTrue(intake.shootCubeCommand());
    driveJoystick.povRight().toggleOnTrue(intake.intakeCommand(()-> 0.4));
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

  void manual(){
    CommandScheduler.getInstance().cancelAll();

//    superstructure.arm.setDefaultCommand(superstructure.manualCommand(driveJoystick::getRightY, () -> driveJoystick.getHID().getPOV(), driveJoystick.triangle()));
//    driveJoystick.R1().toggleOnTrue(superstructure.floatCommand());
  }

  public Command toggleCompressorCommand() {
    return new StartEndCommand(
          compressor::enableDigital,
          compressor::disable
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the comman
   * d to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
