package frc.robot.utility;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Coordinates.GamePiece;
import frc.robot.commands.autonomous.ClimbOverRampCommand;
import frc.robot.commands.autonomous.LeaveCommunityCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.swerve.Swerve;

import javax.print.attribute.standard.PrinterURI;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.ArmConstants.Setpoints.*;

public class AutoBuilder {
    public static final SendableChooser<Command> autoChooser = new SendableChooser<>();
    public static final SendableChooser<Double> heightChooser = new SendableChooser<>();
    public static final SendableChooser<GamePiece> initialGamePiece = new SendableChooser<>();

    public static final Timer autoTimer = new Timer();

    public static void loadAutoChoosers(Swerve swerve, Intake intake) {
        initialGamePiece.setDefaultOption("cone", GamePiece.CONE);
        initialGamePiece.addOption("cube", GamePiece.CUBE);

        heightChooser.setDefaultOption("low", LOW_RPM);
        heightChooser.addOption("mid", MID_RPM);
        heightChooser.addOption("high", HIGH_RPM);

        autoChooser.setDefaultOption("leave community", new LeaveCommunityCommand(swerve, true));
        autoChooser.addOption("balance ramp", swerve.climbCommand(true));
//        autoChooser.addOption("climb over & balance ramp", new ClimbOverRampCommand(swerve, true));
        autoChooser.addOption("collect cube", swerve.driveSwerveWithAngleCommand(()-> 0.4, ()-> 0, ()-> 0, ()-> true)
                .alongWith(intake.intakeCommand(0.4)).withTimeout(5.5));
        autoChooser.addOption("don't drive", new InstantCommand(() -> {}));

        var tab = Shuffleboard.getTab("Autonomous builder");
        tab.add("initial game piece", initialGamePiece).withSize(4, 2).withPosition(8, 1);
        tab.add("height", heightChooser).withSize(4, 2).withPosition(8, 3);
        tab.add("auto", autoChooser).withSize(4, 2).withPosition(8, 5);
    }

    public static Command getAutonomousCommand(Superstructure superstructure, Intake intake, Swerve swerve) {
        return new ProxyCommand(
                new SequentialCommandGroup(
                        new InstantCommand(autoTimer::start),
                        swerve.resetGyroCommand(initialGamePiece.getSelected().equals(GamePiece.CUBE) ? 180 : 0),
                        new InstantCommand(() -> CommandScheduler.getInstance().setDefaultCommand(
                                superstructure.rollergripper, superstructure.rollergripper.holdConeCommand())),
                        new ConditionalCommand(
                                superstructure.arm.resetLengthCommand().andThen(
                                        superstructure.placeOnHeightCommand(heightChooser.getSelected())),
                                intake.shootCubeCommand(heightChooser.getSelected()).withTimeout(2),
                                () -> initialGamePiece.getSelected().equals(GamePiece.CONE)),
                        autoChooser.getSelected()
                                .alongWith(new ConditionalCommand(
                                        new InstantCommand(),
                                        superstructure.arm.lockArmCommand(new Trigger(()-> false)),
                                        superstructure.arm.armLockedTrigger)))
                );
    }
}
