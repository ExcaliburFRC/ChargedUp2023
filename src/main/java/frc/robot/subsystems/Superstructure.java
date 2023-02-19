package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants.Setpoints;
import frc.robot.drivetrain.Swerve;

import java.util.concurrent.atomic.AtomicReference;

import static frc.robot.Constants.ClawConstants.GamePiece;
import static frc.robot.subsystems.LEDs.LedMode.PINK;
import static frc.robot.subsystems.LEDs.LedMode.YELLOW;

public class Superstructure extends SubsystemBase {

    private final Swerve swerve = new Swerve();
    private final Arm arm = new Arm();
    private final Claw claw = new Claw();
    private final Spindexer spindexer = new Spindexer();
    private final Intake intake = new Intake();
    private final LEDs leds = new LEDs();

    public static AtomicReference<GamePiece> currentGamePiece;
    public static AtomicReference<Setpoints> currentSetpoint;

    public Superstructure() {
        currentGamePiece.set(GamePiece.EMPTY);
        currentSetpoint.set(Setpoints.SPINDEXER);

        arm.setDefaultCommand(arm.holdSetpointCommand(Setpoints.spindexer)
                .alongWith(setCurrentSetpoint(Setpoints.SPINDEXER)));
        leds.setDefaultCommand(leds.setColorCommand(leds.getAlliance()));
    }

    private boolean isCone(){
        return currentGamePiece.equals(GamePiece.CONE);
    }

    private Command setCurrentSetpoint(Setpoints setpoint){
        return new InstantCommand(()-> currentSetpoint.set(setpoint));
    }

    public Command intakeFromGroundCommand(Trigger isCone) {
        return new SequentialCommandGroup(
                leds.setColorCommand(isCone.getAsBoolean()? YELLOW: PINK).withTimeout(3),
                claw.openClawCommand(),
                intake.intakeCommand().until(spindexer.beambreakTrigger),
                spindexer.straightenGamePieceCommand(),
                arm.holdSetpointCommand(Setpoints.intake),
                setCurrentSetpoint(Setpoints.INTAKE),
                claw.autoCloseCommand());
    }

    public Command intakeCommand(){
        return new SequentialCommandGroup(
                claw.openClawCommand(),
                intake.intakeCommand().until(spindexer.beambreakTrigger),
                spindexer.straightenGamePieceCommand(),
                new InstantCommand(()-> currentGamePiece.set(spindexer.getCurrentItem())),
                leds.setColorCommand(currentGamePiece.get().equals(GamePiece.CONE)? YELLOW : PINK).withTimeout(3));
    }

    public Command placeOnHighCommand() {
        return new SequentialCommandGroup(
                claw.closeClawCommand(),
                swerve.rotateToGridCommand(),
                arm.holdSetpointCommand(isCone()? Setpoints.HIGH.cone : Setpoints.HIGH.cube),
                Commands.runOnce(()-> currentSetpoint.set(Setpoints.HIGH)),
                claw.openClawCommand(),
                new WaitCommand(0.1));
    }

    public Command placeOnMidCommand() {
        return new SequentialCommandGroup(
                claw.closeClawCommand(),
                swerve.rotateToGridCommand(),
                arm.holdSetpointCommand(isCone()? Setpoints.MID.cone : Setpoints.MID.cube),
                Commands.runOnce(()-> currentSetpoint.set(Setpoints.MID)),
                claw.openClawCommand(),
                new WaitCommand(0.1));
    }

    public Command placeOnLowCommand() {
        return new SequentialCommandGroup(
                claw.closeClawCommand(),
                swerve.rotateToGridCommand(),
                arm.holdSetpointCommand(isCone()? Setpoints.LOW.cone : Setpoints.LOW.cube),
                Commands.runOnce(()-> currentSetpoint.set(Setpoints.LOW)),
                claw.openClawCommand(),
                new WaitCommand(0.1));
    }
}
