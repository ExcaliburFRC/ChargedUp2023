package frc.robot.utility;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CuberConstants.CUBER_ANGLE;
import frc.robot.Constants.CuberConstants.CUBER_VELOCITIY;
import frc.robot.commands.autonomous.ClimbOverRampCommand;
import frc.robot.commands.autonomous.LeaveCommunityCommand;
import frc.robot.subsystems.Cuber;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.Swerve;

import static frc.robot.Constants.LedsConstants.GamePiece;

public class AutoBuilder {
    public static final SendableChooser<Command> autoChooser = new SendableChooser<>();
    public static final SendableChooser<Double> heightChooser = new SendableChooser<>();
    public static final SendableChooser<GamePiece> initialGamePiece = new SendableChooser<>();

    public static final Timer autoTimer = new Timer();

    public static void loadAutoChoosers(Swerve swerve, Cuber cuber) {
        initialGamePiece.setDefaultOption("cone", GamePiece.CONE);
        initialGamePiece.addOption("cube", GamePiece.CUBE);

        heightChooser.setDefaultOption("low", 0.0);
        heightChooser.addOption("mid", 1.0);
        heightChooser.addOption("high", 2.0);

        autoChooser.setDefaultOption("leave community", new LeaveCommunityCommand(swerve, true));
        autoChooser.addOption("balance ramp", swerve.climbCommand(true));
        autoChooser.addOption("climb over & balance ramp", new ClimbOverRampCommand(swerve, getRobotHeading(initialGamePiece.getSelected())));
        autoChooser.addOption("don't drive", new InstantCommand(() -> {}));

        var tab = Shuffleboard.getTab("Autonomous builder");
        tab.add("initial game piece", initialGamePiece).withSize(4, 2).withPosition(8, 1);
        tab.add("height", heightChooser).withSize(4, 2).withPosition(8, 3);
        tab.add("auto", autoChooser).withSize(4, 2).withPosition(8, 5);
    }

    public static Command getAutonomousCommand(Superstructure superstructure, Cuber cuber, Swerve swerve) {
        return new ProxyCommand(
                new SequentialCommandGroup(
                        new InstantCommand(autoTimer::start),
                        swerve.setOdometryAngleCommand(initialGamePiece.getSelected().equals(GamePiece.CUBE) ? 180 : 0),

                        new ConditionalCommand(
                                superstructure.arm.resetLengthCommand().andThen(
                                        superstructure.placeOnHeightCommand(heightChooser.getSelected())),

                                cuber.shootCubeCommand(
                                        getVelocity(heightChooser.getSelected()),
                                        getAngle(heightChooser.getSelected()),
                                        new Trigger(()-> true)),

                                () -> initialGamePiece.getSelected().equals(GamePiece.CONE)),

                        autoChooser.getSelected().alongWith(new ConditionalCommand(
                                        new InstantCommand(),
                                        superstructure.lockArmCommand(),
                                        superstructure.arm.armLockedTrigger)))
        );
    }

    private static CUBER_VELOCITIY getVelocity(double height){
        if (height == 1) return CUBER_VELOCITIY.MIDDLE;
        return CUBER_VELOCITIY.HIGH;
    }

    private static CUBER_ANGLE getAngle(double angle){
        if (angle == 1) return CUBER_ANGLE.MIDDLE;
        return CUBER_ANGLE.HIGH;
    }

    private static boolean getRobotHeading(GamePiece gamePiece){
        return gamePiece.equals(GamePiece.CUBE);
    }
}
