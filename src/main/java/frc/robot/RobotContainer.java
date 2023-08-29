// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LedsConstants.GamePiece;
import frc.robot.subsystems.Cuber;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utility.Calculation;

import java.util.Map;

import static frc.robot.Constants.CuberConstants.CUBER_ANGLE;
import static frc.robot.Constants.CuberConstants.SHOOTER_VELOCITIY;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Cuber cuber = new Cuber();
  private final Swerve swerve = new Swerve();
  private final Superstructure superstructure = new Superstructure();

  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  public final CommandPS4Controller driver = new CommandPS4Controller(0);
  public final CommandPS4Controller operator = new CommandPS4Controller(1);

  public static final ShuffleboardTab driveTab = Shuffleboard.getTab("driveTab");

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();

    // TODO: update AutoBuilder
//    AutoBuilder.loadAutoChoosers(swerve, intake);

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
                () -> Calculation.deadband(-driver.getLeftY()),
                () -> Calculation.deadband(driver.getLeftX()),
                () -> Calculation.deadband(driver.getRightX()),
                driver.R2().negate(),
                driver::getL2Axis));

    // intake commands
    operator.square().toggleOnTrue(superstructure.intakeFromShelfCommand());
    operator.L2().whileTrue(cuber.intakeCommand(CUBER_ANGLE.INTAKE_GROUND));
    operator.R2().whileTrue(cuber.intakeCommand(CUBER_ANGLE.INTAKE_SLIDE));

    // place commands
    operator.triangle().toggleOnTrue(superstructure.placeOnHighCommand(operator.R1()));
    operator.circle().toggleOnTrue(superstructure.placeOnMidCommand(operator.R1()));
    operator.cross().toggleOnTrue(superstructure.placeOnLowCommand());

    operator.povUp().toggleOnTrue(cuber.shootCubeCommand(SHOOTER_VELOCITIY.HIGH, CUBER_ANGLE.HIGH)
                    .alongWith(superstructure.leanBackCommand()));
    operator.povLeft().toggleOnTrue(cuber.shootCubeCommand(SHOOTER_VELOCITIY.MIDDLE, CUBER_ANGLE.MIDDLE));
    operator.povDown().toggleOnTrue(cuber.shootCubeCommand(SHOOTER_VELOCITIY.LOW, CUBER_ANGLE.LOW));

    // other
    driver.touchpad().toggleOnTrue(toggleCompressorCommand());
    driver.PS().onTrue(swerve.resetGyroCommand());

    operator.L1().toggleOnTrue(superstructure.lockArmCommand());
    driver.square().whileTrue(swerve.balanceRampCommand());

    driver.L1().onTrue(askForGamepieceCommand(GamePiece.Cone));
    driver.R1().onTrue(askForGamepieceCommand(GamePiece.Cube));
  }

  public Command toggleCompressorCommand() {
    return new StartEndCommand(
          compressor::disable,
          compressor::enableDigital
    );
  }

  public Command askForGamepieceCommand(GamePiece gamePiece){
    return LEDs.getInstance().applyPatternCommand(LEDs.LEDPattern.BLINKING, gamePiece.color).withTimeout(2);
  }

  public static Command selectDriveTabCommand(){
    return new InstantCommand(()-> Shuffleboard.selectTab(driveTab.getTitle()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
