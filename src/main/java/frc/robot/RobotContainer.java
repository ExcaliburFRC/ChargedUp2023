// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LedsConstants.GamePiece;
import frc.robot.commands.autonomous.ClimbOverRampCommand;
import frc.robot.subsystems.Cuber;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utility.AutoBuilder;
import frc.robot.utility.Calculation;
import frc.robot.utility.Color;
import frc.robot.utility.MorseLEDs;

import java.util.Map;

import static frc.robot.Constants.CuberConstants.CUBER_ANGLE;
import static frc.robot.Constants.CuberConstants.CUBER_VELOCITIY;
import static frc.robot.subsystems.LEDs.LEDPattern.OFF;
import static frc.robot.subsystems.LEDs.LEDPattern.*;
import static frc.robot.utility.Colors.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Cuber cuber = new Cuber();
    private final LEDs leds = LEDs.getInstance();
    private final Superstructure superstructure = new Superstructure();
    private final AutoBuilder autoBuilder = new AutoBuilder(swerve);

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
                        driver.L2().negate(),
                        () -> Calculation.getSwerveDeceleratorVal(driver.getR2Axis()))
        );

        // intake commands
        operator.R1().toggleOnTrue(superstructure.intakeFromShelfCommand());

        operator.L2().whileTrue(cuber.intakeCommand(CUBER_ANGLE.INTAKE_GROUND));
        operator.L1().toggleOnTrue(cuber.intakeCommand(CUBER_ANGLE.INTAKE_SLIDE));

        // shoot / place commands
        operator.triangle().toggleOnTrue(superstructure.placeOnHighCommand(driver.R1()));
        operator.circle().toggleOnTrue(superstructure.placeOnMidCommand(driver.R1()));
        operator.cross().toggleOnTrue(superstructure.placeOnLowCommand());

        operator.povUp().toggleOnTrue(superstructure.adjustForShooterCommand(
                cuber.shootCubeCommand(CUBER_VELOCITIY.HIGH, CUBER_ANGLE.HIGH, driver.R1()), cuber.armSafe));
        operator.povLeft().toggleOnTrue(superstructure.adjustForShooterCommand(
                cuber.shootCubeCommand(CUBER_VELOCITIY.MIDDLE, CUBER_ANGLE.MIDDLE, driver.R1()), cuber.armSafe));
        operator.povDown().toggleOnTrue(cuber.shootCubeCommand(CUBER_VELOCITIY.LOW, CUBER_ANGLE.LOW, driver.R1()));

        operator.povRight().toggleOnTrue(cuber.cannonShooterCommand(swerve::getRobotPitch, driver.R1()));

        // other
        operator.square().onTrue(superstructure.lockArmCommand());
        operator.R2().onTrue(superstructure.arm.forceLockArm());
        operator.button(15).toggleOnTrue(toggleLedsCommand());

        driver.PS().onTrue(swerve.resetOdometryAngleCommand());

        driver.square().whileTrue(swerve.balanceRampCommand());

        driver.povUp().whileTrue(swerve.turnToAngleCommand(0));
        driver.povRight().whileTrue(swerve.turnToAngleCommand(90));
        driver.povDown().whileTrue(swerve.turnToAngleCommand(180));
        driver.povLeft().whileTrue(swerve.turnToAngleCommand(270));

        driver.button(11).onTrue(askForGamepieceCommand(GamePiece.Cone));
        driver.button(12).onTrue(askForGamepieceCommand(GamePiece.Cube));

        driver.touchpad().whileTrue(toggleMotorsIdleMode());
        driver.touchpad().whileTrue(leds.applyPatternCommand(SOLID, WHITE.color));
        driver.button(15).onTrue(MorseLEDs.textToLeds("move it move it", WHITE.color));
    }

    private Command toggleLedsCommand() {
        return new StartEndCommand(
                () -> {
                    leds.restoreLEDs();
                    leds.setDefaultCommand(leds.applyPatternCommand(OFF, new Color()));
                },
                () -> {
                    leds.restoreLEDs();
                    leds.setDefaultCommand(leds.applyPatternCommand(TRAIN_CIRCLE, BLUE.color, TEAM_GOLD.color));
                }).ignoringDisable(true);
    }

    public Command askForGamepieceCommand(GamePiece gamePiece) {
        return leds.applyPatternCommand(LEDs.LEDPattern.BLINKING, gamePiece.color).withTimeout(2);
    }

    public static Command selectDriveTabCommand() {
        return new InstantCommand(() -> Shuffleboard.selectTab(driveTab.getTitle()));
    }

    public Command toggleMotorsIdleMode() {
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
        return new SequentialCommandGroup(
                superstructure.placeOnMidSequentially(),
                superstructure.lockArmCommand(),
                new ClimbOverRampCommand(swerve, true)
        );
    }

        /*  return new ParallelDeadlineGroup(
                autoBuilder.generatePath(List.of(
                        AutoBuilder.getPathpoint(new Translation2d(1.78, 0.50), 0, 0),
                        AutoBuilder.getPathpoint(new Translation2d(7.08, 0.75), -5.24, 0))),
                cuber.intakeCommand(CUBER_ANGLE.INTAKE_GROUND));
    */



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


    /*
    autos:

    collect cube 5:
    AutoBuilder.getPathpoint(new Translation2d(1.78, 4.90), -17.50, 0),
    AutoBuilder.getPathpoint(new Translation2d(5.09, 5.85), 31.26, 43.78)


    new ParallelDeadlineGroup(
                autoBuilder.generatePath(List.of(
                        AutoBuilder.getPathpoint(new Translation2d(1.77, 4.93), -13.94, 0),
                        AutoBuilder.getPathpoint(new Translation2d(7.35, 5.28), -88.60, -62.78),
                        AutoBuilder.getPathpoint(new Translation2d(5.35, 4.81), 168.93, 73.86),
                        AutoBuilder.getPathpoint(new Translation2d(1.77, 4.41), -149.81, 180.00))),
                cuber.intakeCommand(CUBER_ANGLE.INTAKE_GROUND)).withTimeout(5)
                .andThen(
                superstructure.adjustForShooterCommand(
                        cuber.shootCubeCommand(CUBER_VELOCITIY.HIGH, CUBER_ANGLE.HIGH, new Trigger(() -> true)), cuber.armSafe)
        );

     */
}
