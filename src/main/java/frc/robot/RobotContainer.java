// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Coordinates.GamePiece;
import frc.robot.commands.autonomous.LeaveCommunityCommand;
import frc.robot.subsystems.*;
import frc.robot.swerve.Swerve;
import frc.robot.utility.AutoBuilder;

import static frc.robot.Constants.IntakeConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Intake intake = new Intake();
  private final Swerve swerve = new Swerve();
  private final Superstructure superstructure = new Superstructure();

  private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

  public final CommandPS4Controller driveJoystick = new CommandPS4Controller(0);
  public final CommandPS4Controller armJoystick = new CommandPS4Controller(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();

    SmartDashboard.putData("intake", intake);

    var tab = Shuffleboard.getTab("Swerve");
    tab.add("swerve", swerve).withWidget(BuiltInWidgets.kGyro);

    AutoBuilder.loadAutoChoosers(swerve);
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
    swerve.setDefaultCommand(
          swerve.driveSwerveCommand(
                ()-> -driveJoystick.getLeftY(),
                driveJoystick::getLeftX,
                driveJoystick::getRightX,
                driveJoystick.R2().negate()));

    // intake commands
    armJoystick.povRight().toggleOnTrue(intake.intakeCommand(0.4));
    armJoystick.square().toggleOnTrue(superstructure.intakeFromShelfCommand());

    // place commands
    armJoystick.triangle().toggleOnTrue(superstructure.placeOnHighCommand(armJoystick.R1()));
    armJoystick.circle().toggleOnTrue(superstructure.placeOnMidCommand(armJoystick.R1()));
    armJoystick.cross().toggleOnTrue(superstructure.placeOnLowCommand());

    armJoystick.povUp().toggleOnTrue(intake.shootCubeCommand(HIGH_RPM));
    armJoystick.povLeft().toggleOnTrue(intake.shootCubeCommand(MID_RPM));
    armJoystick.povDown().toggleOnTrue(intake.shootCubeToLowCommand());

    // LED control
//    driveJoystick.options().onTrue(askForGamePieceCommand(GamePiece.CONE));
//    driveJoystick.share().onTrue(askForGamePieceCommand(GamePiece.CUBE));

    // other
    driveJoystick.touchpad().toggleOnTrue(toggleCompressorCommand());
    driveJoystick.PS().onTrue(swerve.resetGyroCommand());
    armJoystick.touchpad().whileTrue(intake.orientCubeCommand());
//    armJoystick.button(15).toggleOnTrue(superstructure.arm.lockArmCommand());
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

  public Command toggleCompressorCommand() {
    return new StartEndCommand(
          compressor::disable,
          compressor::enableDigital
          );
  }

  void manual(){
    superstructure.arm.setDefaultCommand(
    superstructure.arm.joystickManualCommand(armJoystick::getLeftY, armJoystick::getRightY)
    );

    superstructure.rollerGripper.setDefaultCommand(
          superstructure.rollerGripper.manualCommand(armJoystick.square(), armJoystick.circle())
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the comman
   * d to run in autonomous
   */
  public Command getAutonomousCommand() {
    return AutoBuilder.getAutonomousCommand(superstructure, intake);
  }
}
