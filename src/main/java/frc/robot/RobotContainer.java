// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.swerve.Swerve;
import frc.robot.utility.Calculation;

import java.util.HashMap;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public final Swerve swerve = new Swerve();

  public final CommandPS4Controller driveJoystick = new CommandPS4Controller(0);
  public final CommandPS4Controller armJoystick = new CommandPS4Controller(1);

  public static final ShuffleboardTab driveTab = Shuffleboard.getTab("driveTab");

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
                driveJoystick.R1().negate(),
                driveJoystick.L1()));

    driveJoystick.PS().onTrue(swerve.resetGyroCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the comman
   * d to run in autonomous
   */
  public Command getAutonomousCommand() {
      PathPlannerTrajectory trajectory = PathPlanner.loadPath("RightTurn",new PathConstraints(1,1));

    return swerve.resetModulesCommand().andThen(
          new InstantCommand(()-> {
            swerve.resetGyro();
            swerve.setPose2d(new Pose2d(0, 0, new Rotation2d(0)));
          }),
          swerve.resetGyroCommand(),
          swerve.followTrajectoryCommand(trajectory));
  }
}
