// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import  edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SystemTester;
import frc.robot.subsystems.*;
import frc.robot.swerve.Swerve;
import frc.robot.utility.AutoBuilder;
import frc.robot.utility.Calculation;

import java.util.Map;

import static frc.robot.Constants.IntakeConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Intake intake = new Intake();
  public final Swerve swerve = new Swerve();
  private final Superstructure superstructure = new Superstructure();

  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  public final CommandPS4Controller driveJoystick = new CommandPS4Controller(0);
  public final CommandPS4Controller armJoystick = new CommandPS4Controller(1);

  public static final ShuffleboardTab driveTab = Shuffleboard.getTab("driveTab");

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();
    AutoBuilder.loadAutoChoosers(swerve, intake);

    SmartDashboard.putData("intake", intake);

    driveTab.addDouble("Remaining Time", DriverStation::getMatchTime)
            .withSize(4, 4).withPosition(16, 0).withWidget("Simple Dial")
          .withProperties(Map.of("min", 0, "max", 135));
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
                () -> Calculation.deadband(-driveJoystick.getLeftY()),
                () -> Calculation.deadband(driveJoystick.getLeftX()),
                () -> Calculation.deadband(driveJoystick.getRightX()),
                driveJoystick.R1().negate(),
                driveJoystick.L1()));

    intake.setDefaultCommand(
          intake.setIntakeSpeedCommand(0.05).withTimeout(1.25)
                .andThen(new RunCommand(()-> {}, intake)));

    // intake commands
    armJoystick.povRight().toggleOnTrue(intake.intakeCommand(0.45));
    armJoystick.square().toggleOnTrue(superstructure.intakeFromShelfCommand());

    armJoystick.PS().toggleOnTrue(intake.togglePistonCommand());

    // place commands
    armJoystick.triangle().toggleOnTrue(superstructure.placeOnHighCommand(armJoystick.R1()));
    armJoystick.circle().toggleOnTrue(superstructure.placeOnMidCommand(armJoystick.R1()));
    armJoystick.cross().toggleOnTrue(superstructure.placeOnLowCommand());

    armJoystick.povUp().toggleOnTrue(intake.shootCubeCommand(HIGH_RPM));
    armJoystick.povLeft().toggleOnTrue(intake.shootCubeCommand(MID_RPM));
    armJoystick.povDown().toggleOnTrue(intake.shootCubeToLowCommand());

    armJoystick.L2().whileTrue(intake.collectCommand());
    armJoystick.R2().whileTrue(intake.letoutCommand());

    // other
    driveJoystick.touchpad().toggleOnTrue(toggleCompressorCommand());
    driveJoystick.PS().onTrue(swerve.resetGyroCommand());
    armJoystick.touchpad().whileTrue(intake.intakeFromSlideCommand());

    armJoystick.L1().toggleOnTrue(superstructure.lockArmCommand());
    driveJoystick.square().whileTrue(swerve.balanceRampCommand());

    driveJoystick.button(15).onTrue(swerve.resetModulesCommand());
    armJoystick.button(15).whileTrue(superstructure.arm.blindLockArmCommand());
  }

  public Command toggleCompressorCommand() {
    return new StartEndCommand(
          compressor::disable,
          compressor::enableDigital
    );
  }

  Command SystemTester() {
    return new SystemTester(swerve, intake, superstructure.rollergripper);
  }

  public static Command selectDriveTabCommand(){
    return new InstantCommand(()-> Shuffleboard.selectTab(driveTab.getTitle()));
  }

  Command manualArm(){
    return superstructure.arm.joystickManualCommand(armJoystick::getLeftY, armJoystick::getRightY)
          .alongWith(superstructure.rollergripper.manualCommand(armJoystick.square(), armJoystick.circle()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the comman
   * d to run in autonomous
   */
  public Command getAutonomousCommand() {
    return AutoBuilder.getAutonomousCommand(superstructure, intake, swerve);
  }
}
