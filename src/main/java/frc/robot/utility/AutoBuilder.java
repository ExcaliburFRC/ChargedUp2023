package frc.robot.utility;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.Constants;
import frc.robot.commands.autonomous.LeaveCommunityCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.swerve.Swerve;

import static frc.robot.Constants.IntakeConstants.*;

public class AutoBuilder {
  public static final SendableChooser<Command> autoChooser = new SendableChooser<>();
  public static final SendableChooser<Double> heightChooser = new SendableChooser<>();
  public static final SendableChooser<Constants.Coordinates.GamePiece> initialGamePiece = new SendableChooser<>();
  public static final SendableChooser<Boolean> facingChooser = new SendableChooser<>();

  public static void loadAutoChoosers(Swerve swerve){
    initialGamePiece.setDefaultOption("cone", Constants.Coordinates.GamePiece.CONE);
    initialGamePiece.addOption("cube", Constants.Coordinates.GamePiece.CUBE);

    heightChooser.setDefaultOption("low", LOW_RPM);
    heightChooser.addOption("mid", MID_RPM);
    heightChooser.addOption("high", HIGH_RPM);

    facingChooser.setDefaultOption("forward (place cone)", true);
    facingChooser.addOption("backwards (place cube)", false);

    autoChooser.setDefaultOption("leave community", new LeaveCommunityCommand(swerve));
    autoChooser.addOption("balance ramp", swerve.climbCommand(false));
    autoChooser.addOption("don't drive", new InstantCommand(()-> {}));

    var tab = Shuffleboard.getTab("Autonomous builder");
//    tab.add("initial game piece", initialGamePiece).withSize(3, 1)
//          .withPosition(5, 1);
    tab.add("height", heightChooser).withSize(3, 1).withPosition(5, 2);
//    tab.add("facing forwards", facingChooser).withSize(3, 1)
//          .withPosition(5, 3);
    tab.add("auto", autoChooser).withSize(3, 1).withPosition(5, 3); // 5, 4
  }

//  public static Command getAutonomousCommand(Superstructure superstructure, Intake intake){
//    return new ProxyCommand(
//           cone or cube
//          new ConditionalCommand(
//                superstructure.switchCommand(heightChooser.getSelected()),
//                intake.shootCubeCommand(heightChooser.getSelected()),
//                () -> initialGamePiece.getSelected().equals(Constants.Coordinates.GamePiece.CONE))
//                 leave or climb
//                .andThen(autoChooser.getSelected()));
//  }

  public static Command getAutonomousCommand(Intake intake, Swerve swerve){
    return new ProxyCommand(
          new InstantCommand(()-> swerve.resetGyroCommand(180)).andThen(
                intake.shootCubeCommand(heightChooser.getSelected()).withTimeout(2),
                autoChooser.getSelected()));
  }

}
