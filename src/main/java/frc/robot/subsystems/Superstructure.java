package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants.Setpoints;
import frc.robot.drivetrain.Swerve;

import java.util.concurrent.atomic.AtomicReference;

import static frc.robot.Constants.ClawConstants.GamePiece;
import static frc.robot.subsystems.LEDs.LedMode.*;

public class Superstructure extends SubsystemBase {

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
    private Command setCurrentGamePiece(GamePiece gamePiece){
        return new InstantCommand(()-> currentGamePiece.set(gamePiece));
    }

    public Command intakeFromClawCommand() {
        return new SequentialCommandGroup(
                claw.openClawCommand(),
                arm.holdSetpointCommand(Setpoints.intake),
                setCurrentSetpoint(Setpoints.INTAKE),
                claw.autoCloseCommand());
    }

    public Command intakeCommand(){
        return new SequentialCommandGroup(
                claw.openClawCommand(),
                intake.intakeCommand().until(spindexer.beambreakTrigger),
                spindexer.straightenGamePieceCommand(),
                setCurrentGamePiece(spindexer.getCurrentGamePiece()),
                leds.setColorCommand(currentGamePiece.get().equals(GamePiece.CONE)? ORANGE : PURPLE).withTimeout(3));
    }

    public Command placeOnHighCommand() {
        return new SequentialCommandGroup(
                claw.closeClawCommand(),
                arm.holdSetpointCommand(isCone()? Setpoints.HIGH.cone : Setpoints.HIGH.cube),
                setCurrentSetpoint(Setpoints.HIGH),
                claw.openClawCommand(),
                new WaitCommand(0.1));
    }

    public Command placeOnMidCommand() {
        return new SequentialCommandGroup(
                claw.closeClawCommand(),
                arm.holdSetpointCommand(isCone()? Setpoints.MID.cone : Setpoints.MID.cube),
                setCurrentSetpoint(Setpoints.MID),
                claw.openClawCommand(),
                new WaitCommand(0.1));
    }

    public Command placeOnLowCommand() {
        return new SequentialCommandGroup(
                claw.closeClawCommand(),
                arm.holdSetpointCommand(isCone()? Setpoints.LOW.cone : Setpoints.LOW.cube),
                setCurrentSetpoint(Setpoints.LOW),
                claw.openClawCommand(),
                new WaitCommand(0.1));
    }

    public Command askFroGamePieceCommand(GamePiece gamePiece){
        return Commands.repeatingSequence(
                leds.setColorCommand(gamePiece.equals(GamePiece.CONE) ? ORANGE : PURPLE),
                new WaitCommand(0.5),
                leds.setColorCommand(OFF),
                new WaitCommand(0.5))
                .withTimeout(5);
    }
}
