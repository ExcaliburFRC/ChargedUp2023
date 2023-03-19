package frc.robot.utility;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.Constants.Coordinates.GamePiece;
import frc.robot.commands.autonomous.LeaveCommunityCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.swerve.Swerve;

import static frc.robot.Constants.IntakeConstants.*;

public class AutoBuilder {
  public static final SendableChooser<Command> autoChooser = new SendableChooser<>();
  public static final SendableChooser<Double> heightChooser = new SendableChooser<>();
  public static final SendableChooser<GamePiece> initialGamePiece = new SendableChooser<>();

  public static void loadAutoChoosers(Swerve swerve){
    initialGamePiece.setDefaultOption("cone", GamePiece.CONE);
    initialGamePiece.addOption("cube", GamePiece.CUBE);

    heightChooser.setDefaultOption("low", LOW_RPM);
    heightChooser.addOption("mid", MID_RPM);
    heightChooser.addOption("high", HIGH_RPM);

    autoChooser.setDefaultOption("leave community", new LeaveCommunityCommand(swerve, true));
    autoChooser.addOption("balance ramp", swerve.climbCommand(true));
    autoChooser.addOption("don't drive", new InstantCommand(()-> {}));

    var tab = Shuffleboard.getTab("Autonomous builder");
    tab.add("initial game piece", initialGamePiece).withSize(3, 1)
          .withPosition(5, 1);
    tab.add("height", heightChooser).withSize(3, 1).withPosition(5, 2);
    tab.add("auto", autoChooser).withSize(3, 1).withPosition(5, 3); // 5, 4
  }

  public static Command getAutonomousCommand(Superstructure superstructure, Intake intake, Swerve swerve){
    return new ProxyCommand(
//           cone or cube
          swerve.resetGyroCommand(initialGamePiece.equals(GamePiece.CUBE)? 0 : 180)
                .andThen(
                      new PrintCommand("gyro angle: " + swerve.getGyroRotation().getDegrees()),
          new ConditionalCommand(
                superstructure.switchCommand(heightChooser.getSelected()),
                intake.shootCubeCommand(heightChooser.getSelected()).withTimeout(2),
                () -> initialGamePiece.getSelected().equals(GamePiece.CONE)),
//                 leave or climb
                 autoChooser.getSelected()));
  }
//
//  public static Command getAutonomousCommand(Intake intake, Swerve swerve){
//    return new ProxyCommand(
//          new InstantCommand(()-> swerve.resetGyroCommand(180)).andThen(
//                intake.shootCubeCommand(heightChooser.getSelected()).withTimeout(2),
//                autoChooser.getSelected()));
//  }
}
