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
    public final SendableChooser<Command> autoChooser = new SendableChooser<>();
//    public final SendableChooser<Double> heightChooser = new SendableChooser<>();
    public final SendableChooser<GamePiece> initialGamePiece = new SendableChooser<>();

    private final Swerve swerve;
    private final Cuber cuber;
    private final Superstructure superstructure;

    public AutoBuilder(Swerve swerve, Cuber cuber, Superstructure superstructure){
        this.swerve = swerve;
        this.cuber = cuber;
        this.superstructure = superstructure;
    }

    public static final Timer autoTimer = new Timer();

    public void loadAutoChoosers() {
        initialGamePiece.setDefaultOption("cone", GamePiece.CONE);
        initialGamePiece.addOption("cube", GamePiece.CUBE);

//        heightChooser.setDefaultOption("low", 0.0);
//        heightChooser.addOption("mid", 1.0);
//        heightChooser.addOption("high", 2.0);

        autoChooser.setDefaultOption("leave community", new LeaveCommunityCommand(swerve, getRobotHeading(initialGamePiece.getSelected())));
        autoChooser.addOption("balance ramp", swerve.climbCommand(true, 0));
        autoChooser.addOption("climb over & balance ramp", new ClimbOverRampCommand(swerve, getRobotHeading(initialGamePiece.getSelected())));
        autoChooser.addOption("don't drive", new InstantCommand(() -> {}));

        var tab = Shuffleboard.getTab("Autonomous builder");
        tab.add("initial game piece", initialGamePiece).withSize(4, 2).withPosition(8, 1);
//        tab.add("height", heightChooser).withSize(4, 2).withPosition(8, 3);
        tab.add("auto", autoChooser).withSize(4, 2).withPosition(8, 3);
    }

    public Command getAutonomousCommand() {
        return new ProxyCommand(
                new SequentialCommandGroup(
                        //init
                        new InstantCommand(autoTimer::start),
//                        new InstantCommand(LEDs.getInstance()::restoreLEDs),
                        swerve.setOdometryAngleCommand(initialGamePiece.getSelected().equals(GamePiece.CUBE) ? 180 : 0),

                        // place cone / cube
                        new ConditionalCommand(
                                superstructure.placeOnMidSequentially(),
                                cuber.shootCubeCommand(CUBER_VELOCITIY.MIDDLE, CUBER_ANGLE.MIDDLE, new Trigger(()-> true)),
                                () -> initialGamePiece.getSelected().equals(GamePiece.CONE)),

                        // lock arm if placed cone
                        new ConditionalCommand(
                                new InstantCommand(),
                                superstructure.arm.lockArmWithSetpoint(),
                                superstructure.arm.armLockedTrigger),

                        // drive auto
                        autoChooser.getSelected())
                );
    }

    private static boolean getRobotHeading(GamePiece gamePiece){
        return gamePiece.equals(GamePiece.CONE);
    }
}
