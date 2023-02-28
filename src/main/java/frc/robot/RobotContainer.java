// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.autonomous.noRamsete.EmptyCommand;
import frc.robot.commands.autonomous.noRamsete.cube.CubeAndClimb;
import frc.robot.commands.autonomous.noRamsete.cube.CubeAndLeave;
import frc.robot.commands.autonomous.noRamsete.noGamePiece.ClimbRamp;
import frc.robot.commands.autonomous.noRamsete.noGamePiece.LeaveCommunity;
import frc.robot.subsystems.*;
import frc.robot.swerve.Swerve;
import frc.robot.utiliy.Calculation;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
//  private final Superstructure superstructure = new Superstructure();
  private final Intake intake = new Intake();
  private final Swerve swerve = new Swerve();
  private final Arm arm = new Arm();
  private final RollerGripper rollerGripper = new RollerGripper();

  private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

  public final SendableChooser<Command> autoChooser = new SendableChooser();
  public final SendableChooser<Integer> heightChooser = new SendableChooser();
  public final SendableChooser<Integer> facingChooser = new SendableChooser();

  public final CommandPS4Controller driveJoystick = new CommandPS4Controller(0);
  public final CommandPS4Controller armJoystick = new CommandPS4Controller(1);

  public double offset = 0;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();
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
    // auto configuration
    heightChooser.setDefaultOption("1", new Integer(1));
    heightChooser.addOption("2", new Integer(2));
    heightChooser.addOption("3", new Integer(3));

    autoChooser.setDefaultOption("LeaveCommunity", new LeaveCommunity(swerve));
    autoChooser.addOption("ClimbRamp", new ClimbRamp(swerve));

    autoChooser.addOption("CubeAndLeave", new CubeAndLeave(swerve, intake,3));
    autoChooser.addOption("CubeAndClimb", new CubeAndClimb(swerve, intake, 3));

    autoChooser.addOption("empty", new EmptyCommand());

    facingChooser.setDefaultOption("forward", 0);
    facingChooser.addOption("backwards", 180);

    SmartDashboard.putData(autoChooser);
    SmartDashboard.putData(heightChooser);

//    leds.setDefaultCommand(leds.setColorCommand(leds.getAlliance()));

    swerve.setDefaultCommand(
          swerve.driveSwerveCommand(
                ()-> -driveJoystick.getLeftY(),
                driveJoystick::getLeftX,
                driveJoystick::getRightX,
                driveJoystick.R2().negate()));

    // intake commands
//    armJoystick.square().toggleOnTrue(superstructure.intakeFromShelfCommand(driveJoystick.L1(), driveJoystick.R1()));
    armJoystick.povRight().toggleOnTrue(intake.intakeCommand(0.4)); //, leds

    // place commands
//    armJoystick.triangle().onTrue(superstructure.placeOnHighCommand(driveJoystick.R2(), driveJoystick.L1(), driveJoystick.R1(),  armJoystick::getRightY));
//    armJoystick.circle().onTrue(superstructure.placeOnMidCommand(driveJoystick.R2(), driveJoystick.L1(), driveJoystick.R1(), armJoystick::getRightY));
//    armJoystick.cross().onTrue(superstructure.placeOnLowCommand(driveJoystick.R2(), driveJoystick.L1(), driveJoystick.R1(), armJoystick::getRightY));

    armJoystick.povUp().toggleOnTrue(intake.shootCubeCommand(3, ()-> offset));
    armJoystick.povLeft().toggleOnTrue(intake.shootCubeCommand(2, ()-> offset));
    armJoystick.povDown().toggleOnTrue(intake.shootCubeCommand(1, ()-> offset));

    // LED control
//    driveJoystick.options().onTrue(askForGamePieceCommand(GamePiece.CONE));
//    driveJoystick.share().onTrue(askForGamePieceCommand(GamePiece.CUBE));

    // other
    driveJoystick.touchpad().toggleOnTrue(toggleCompressorCommand());
    driveJoystick.PS().onTrue(swerve.resetGyroCommand());
//    new Trigger(()-> armJoystick.getHID().getRawButtonPressed(15))
//          .toggleOnTrue(superstructure.arm.lowerTelescopeCommand());

    driveJoystick.povUp().onTrue(offsetShooter(0.05));
    driveJoystick.povDown().onTrue(offsetShooter(-0.05));

    arm.setDefaultCommand(
          arm.joystickManualCommand(() -> Calculation.deadband(armJoystick.getLeftY(), 0.1), armJoystick::getRightY)
    );

    armJoystick.R1().toggleOnTrue(rollerGripper.intakeCommand());
    armJoystick.L1().toggleOnTrue(rollerGripper.ejectCommand());

    rollerGripper.setDefaultCommand(
          rollerGripper.holdConeCommand()
    );
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
          compressor::enableDigital,
          compressor::disable
    );
  }

  public Command offsetShooter(double offset){
    return new InstantCommand(()-> this.offset -= offset);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the comman
   * d to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
