package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.concurrent.atomic.AtomicReference;

import static frc.robot.Constants.ClawConstants.GamePiece;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.IntakeConstants.*;

public class Spindexer extends SubsystemBase {

    private final CANSparkMax spindexer = new CANSparkMax(k_SPINDEXER_MOTOR_ID, kBrushless);

    private final DigitalInput button = new DigitalInput(BUTTON_CHANNEL);
    private final DigitalInput beambreak = new DigitalInput(BEAMBREAK_CHANNEL);

    // sensor triggers
    public final Trigger beambreakTrigger = new Trigger(() -> !beambreak.get()).debounce(0.1);
    public final Trigger buttonTrigger = new Trigger(() -> !button.get()).debounce(0.1);

    //state triggers
    private final Trigger ConeStraightTrigger = buttonTrigger.and(beambreakTrigger.negate());
    private final Trigger isConeStuckTrigger = buttonTrigger.and(beambreakTrigger);
    private final Trigger isCubeTrigger = beambreakTrigger.and(buttonTrigger.negate());

    public AtomicReference<GamePiece> currentPiece;

    public Spindexer() {
        spindexer.restoreFactoryDefaults();
        spindexer.clearFaults();
        spindexer.setSmartCurrentLimit(k_DJ_MOTOR_CURRENT_LIMIT);
        spindexer.setIdleMode(CANSparkMax.IdleMode.kBrake);
        spindexer.setInverted(false); //TODO: check

        currentPiece.set(GamePiece.EMPTY);
    }

    public Command straightenGamePieceCommand() {
        return new SequentialCommandGroup(
                setSpindexerMotor(0.3)
                        .until(beambreakTrigger.debounce(3)),
                new ConditionalCommand(
                        handleCubeCommand(),
                        handleConeCommand(),
                        isCubeTrigger),
                setSpindexerMotor(0));
    }

    private Command handleCubeCommand() {
        return setGamePieceCommand(GamePiece.CUBE);
    }

    private Command handleConeCommand() {
        return new SequentialCommandGroup(
                new WaitUntilCommand(buttonTrigger),
                new ConditionalCommand(
                        setSpindexerMotor(0),
                        streightenConeCommand(),
                        ConeStraightTrigger)
                        .andThen(setGamePieceCommand(GamePiece.CONE))
        );
    }

    private ParallelRaceGroup streightenConeCommand() {
        return Commands.repeatingSequence(
                        setSpindexerMotor(-0.3),
                        new WaitCommand(2), // TODO: find the shortest duration for a successful straighten
                        setSpindexerMotor(0.3))
                .until(ConeStraightTrigger);
    }

    private Command setGamePieceCommand(GamePiece gamePiece){
        return Commands.runOnce(()-> currentPiece.set(gamePiece));
    }

    public GamePiece getCurrentItem() {
        return currentPiece.get();
    }

    private Command setSpindexerMotor(double speed) {
        return new RunCommand(() -> spindexer.set(speed), this);
    }
}
