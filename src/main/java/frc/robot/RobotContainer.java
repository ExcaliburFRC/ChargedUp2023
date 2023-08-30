// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.swerve.Swerve;
import frc.robot.utility.AutoBuilder;
import frc.robot.utility.Calculation;

import static frc.robot.utility.AutoBuilder.getPathpoint;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public final Swerve swerve = new Swerve();
  public final AutoBuilder autoBuilder = new AutoBuilder(swerve);

  public final CommandPS4Controller driveJoystick = new CommandPS4Controller(0);
  public final CommandPS4Controller armJoystick = new CommandPS4Controller(1);

  public static final ShuffleboardTab driveTab = Shuffleboard.getTab("driveTab");
  public static final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

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
    swerve.setDefaultCommand(
          swerve.driveSwerveCommand(
                () -> Calculation.deadband(-driveJoystick.getLeftY()),
                () -> Calculation.deadband(driveJoystick.getLeftX()),
                () -> Calculation.deadband(driveJoystick.getRightX()),
                driveJoystick.R1().negate()));

    driveJoystick.PS().onTrue(swerve.resetGyroCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    PathPlannerTrajectory traj = PathPlanner.generatePath(
            new PathConstraints(1, 1),

            getPathpoint(new Translation2d(7.54, 3.14), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
            getPathpoint(new Translation2d(8.51, 3.17), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90)),
            getPathpoint(new Translation2d(7.6, 4), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180)),
            getPathpoint(new Translation2d(6.5, 3.26), Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(-90))
            );

    return autoBuilder.followTrajectoryCommand(traj);
  }
}
