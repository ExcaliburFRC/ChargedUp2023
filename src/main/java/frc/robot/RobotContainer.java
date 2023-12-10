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
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CuberConstants.CUBER_ANGLE;
import frc.robot.Constants.CuberConstants.CUBER_VELOCITIY;
import frc.robot.Constants.LedsConstants.GamePiece;
import frc.robot.subsystems.Cuber;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utility.Calculation;
import frc.robot.utility.Color;
import frc.robot.utility.MorseLEDs;

import java.util.Map;

import static frc.robot.subsystems.LEDs.LEDPattern.OFF;
import static frc.robot.subsystems.LEDs.LEDPattern.*;
import static frc.robot.utility.Color.Colors.*;

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


    public final Trigger userButtonTrigger = new Trigger(RobotController::getUserButton);

    public final CommandPS4Controller driver = new CommandPS4Controller(0);
    public final CommandPS4Controller operator = new CommandPS4Controller(1);
//    public final CommandPS4Controller testController = new CommandPS4Controller(2);

    public static final ShuffleboardTab driveTab = Shuffleboard.getTab("driveTab");

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureBindings();
//        autoBuilder.loadAutoChoosers();

        driveTab.addDouble("Remaining Time", DriverStation::getMatchTime)
                .withSize(8, 8).withPosition(16, 0).withWidget("Simple Dial")
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
                        () -> Calculation.deadband(driver.getLeftY()),
                        () -> Calculation.deadband(driver.getLeftX()),
                        () -> Calculation.deadband(driver.getRightX()),
                        ()-> true,
                        () -> Calculation.getSwerveDeceleratorVal(driver.getR2Axis()),
//                        ()-> getAngleFromButtons(driver.triangle(), driver.circle(), driver.cross(), driver.square()))
                        ()-> -1)
        );

        driver.L2().whileTrue(swerve.straightenModulesCommand());

        // intake commands
        operator.R1().toggleOnTrue(superstructure.intakeFromShelfCommand());

//        operator.L2().whileTrue(cuber.intakeCommand(CUBER_ANGLE.INTAKE_GROUND));
        operator.L1().toggleOnTrue(cuber.intakeCommand(CUBER_ANGLE.INTAKE_SLIDE));


        superstructure.arm.setDefaultCommand(
                superstructure.manualArmCOmmand(operator::getRightY)
        );

        // shoot / place commands
        operator.triangle().toggleOnTrue(superstructure.placeOnHighCommand(driver.R1()));
        operator.circle().toggleOnTrue(superstructure.placeOnMidCommand(driver.R1()));
        operator.cross().toggleOnTrue(superstructure.placeOnLowCommand());

        operator.povUp().toggleOnTrue(superstructure.adjustForShooterCommand(
                cuber.shootCubeCommand(CUBER_VELOCITIY.HIGH, CUBER_ANGLE.HIGH, driver.R1()), cuber.armSafe));
        operator.povLeft().toggleOnTrue(superstructure.adjustForShooterCommand(
                cuber.shootCubeCommand(CUBER_VELOCITIY.MIDDLE, CUBER_ANGLE.MIDDLE, driver.R1()), cuber.armSafe));
        operator.povDown().toggleOnTrue(cuber.shootCubeToLowerCommand(driver.R1()));

        operator.povRight().toggleOnTrue(cuber.cannonShooterCommand(swerve::getRobotPitch, driver.R1()));

        // other
        operator.square().onTrue(superstructure.lockArmCommand());
        operator.R2().onTrue(superstructure.arm.forceLockArm());
        operator.button(15).toggleOnTrue(toggleLedsCommand());

        driver.PS().onTrue(swerve.resetOdometryAngleCommand());
        driver.square().whileTrue(swerve.balanceRampCommand());

        driver.button(11).onTrue(askForGamepieceCommand(GamePiece.CONE));
        driver.button(12).onTrue(askForGamepieceCommand(GamePiece.CUBE));

        driver.touchpad().whileTrue(toggleMotorsIdleMode());
        driver.touchpad().whileTrue(leds.applyPatternCommand(SOLID, WHITE.color));
        driver.button(15).onTrue(MorseLEDs.textToAddressableLeds("excalibur 6738", WHITE.color));
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
//                cuber.toggleIdleModeCommand(),
                superstructure.arm.toggleIdleModeCommand()
        );
    }

    public double getAngleFromButtons(Trigger triangle, Trigger circle, Trigger cross, Trigger square){
        if (triangle.getAsBoolean()) return 0;
        if (circle.getAsBoolean()) return 90;
        if (cross.getAsBoolean()) return 90;
        if (square.getAsBoolean()) return 90;
        return -1;
    }

//    public Command SystemTester(){
//        return new SystemTester(swerve, cuber, superstructure, operator.touchpad());
//    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
