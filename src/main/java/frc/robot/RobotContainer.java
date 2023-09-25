// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LedsConstants.GamePiece;
import frc.robot.subsystems.Cuber;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utility.Calculation;
import frc.robot.utility.Colors;

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
  private final Swerve swerve = new Swerve();
  private final Cuber cuber = new Cuber();
  private final LEDs lEDs = LEDs.getInstance();
  private final Superstructure superstructure = new Superstructure();

  public final Trigger userButtonTrigger = new Trigger(RobotController::getUserButton);

  public final CommandPS4Controller driver = new CommandPS4Controller(0);
  public final CommandPS4Controller operator = new CommandPS4Controller(1);
  public final CommandPS4Controller testController = new CommandPS4Controller(2);

  public static final ShuffleboardTab driveTab = Shuffleboard.getTab("driveTab");

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();

//     TODO: update AutoBuilder
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
                  () -> Calculation.getSwerveDeceleratorVal(driver.getL2Axis()))
    );

    // intake commands
    operator.R1().toggleOnTrue(superstructure.intakeFromShelfCommand());
    operator.L2().whileTrue(cuber.intakeCommand(CUBER_ANGLE.INTAKE_GROUND));
    operator.L1().whileTrue(superstructure.adjustForShooterCommand(
            cuber.intakeCommand(CUBER_ANGLE.INTAKE_SLIDE)));

    // shoot / place commands
    operator.triangle().toggleOnTrue(superstructure.placeOnHighCommand(driver.R1()));
    operator.circle().toggleOnTrue(superstructure.placeOnMidCommand(driver.R1()));
    operator.cross().toggleOnTrue(superstructure.placeOnLowCommand());

    operator.povUp().toggleOnTrue(superstructure.adjustForShooterCommand(
            cuber.shootCubeCommand(SHOOTER_VELOCITIY.HIGH, CUBER_ANGLE.HIGH, driver.R1())));
    operator.povLeft().toggleOnTrue(superstructure.adjustForShooterCommand(
                    cuber.shootCubeCommand(SHOOTER_VELOCITIY.MIDDLE, CUBER_ANGLE.MIDDLE, driver.R1())));
    operator.povDown().toggleOnTrue(
            cuber.shootCubeCommand(SHOOTER_VELOCITIY.LOW, CUBER_ANGLE.LOW_SHOOTER, driver.R1()));

    operator.povRight().toggleOnTrue(cuber.cannonShooterCommand(swerve::getRobotPitch, driver.R1()));

    // other
    // ensures that the cube is fully inside the system
    operator.L2().onFalse(cuber.confirmCubeIntake().withTimeout(2));
    // makes sure that the arm is being locked even if the command was interrupted
    operator.L1().or(operator.povLeft().or(operator.povUp())).onFalse(superstructure.lockArmCommand());
    operator.square().onTrue(superstructure.lockArmCommand());

    driver.PS().onTrue(swerve.resetOdometryAngleCommand());
    driver.square().whileTrue(swerve.balanceRampCommand());

    driver.button(11).onTrue(askForGamepieceCommand(GamePiece.Cone));
    driver.button(12).onTrue(askForGamepieceCommand(GamePiece.Cube));

    driver.touchpad().whileTrue(lEDs.applyPatternCommand(LEDs.LEDPattern.SOLID, Colors.WHITE.color));
    driver.touchpad().whileTrue(toggleMotorsIdleMode());

    cuber.setDefaultCommand(cuber.angleControl(operator::getRightY));
  }

  public Command askForGamepieceCommand(GamePiece gamePiece){
    return lEDs.applyPatternCommand(LEDs.LEDPattern.BLINKING, gamePiece.color).withTimeout(2);
  }

  public static Command selectDriveTabCommand(){
    return new InstantCommand(()-> Shuffleboard.selectTab(driveTab.getTitle()));
  }

  public Command toggleMotorsIdleMode(){
    return new ParallelCommandGroup(
            swerve.toggleIdleModeCommand(),
            cuber.toggleIdleModeCommand(),
            superstructure.arm.toggleIdleModeCommand()
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

  /*
  button layout:

  driver:
  joysticks - drive
  joystick buttons - askForGamePiece LEDs

  R1 - release / confirm

  L2 & R2 - swerve modifications

  Playstation - resetGyro
  Touchpad - Coast motors & whiteLEDs

  square - balanceRamp
  ----------------------------------
  operator:

  X - Lock Arm
  Y, B, A - raise arm to high, mid, low

  RB - intake cone (shelf only)

  LT & LB - intake cube from ground / slide

  POV up, left, down - prepare shooter for high, mid, low
  POV right - enable automatic cannon mode
   */
}
