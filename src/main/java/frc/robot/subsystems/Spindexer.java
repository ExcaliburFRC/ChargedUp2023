package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.ClawConstants.GamePiece;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.IntakeConstants.*;

public class Spindexer extends SubsystemBase {

    private final CANSparkMax spindexer = new CANSparkMax(k_SPINDEXER_MOTOR_ID, kBrushless);
    private final DigitalInput button = new DigitalInput(BUTTON_CHANNEL);
    private final DigitalInput beamBreaker = new DigitalInput(BEAMBREAK_CHANNEL);
    private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kMXP);

    private final Trigger beamBreakDetectedTrigger = new Trigger(() -> !beamBreaker.get());
    private final Trigger buttonDetectedTrigger = new Trigger(() -> !button.get());

    public GamePiece currentPiece;

    public Spindexer() {
        spindexer.restoreFactoryDefaults();
        spindexer.clearFaults();
        spindexer.setSmartCurrentLimit(k_DJ_MOTOR_CURRENT_LIMIT);
        spindexer.setIdleMode(CANSparkMax.IdleMode.kBrake);
        spindexer.setInverted(false); //TODO: check
    }

    public Command straightenGamePieceCommand() {
        return new SequentialCommandGroup(
                setSpindexerMotor(0.3),
                new ConditionalCommand(
                        new InstantCommand(),
                        handleGamePiecesCommand(),
                        () -> getCurrentItem() == GamePiece.EMPTY));
    }

    private Command handleGamePiecesCommand() {
        return new ConditionalCommand(
                handleConeCommand(),
                handleCubeCommand(),
                () -> getCurrentItem().equals(GamePiece.CONE)
        );
    }

    private Command handleCubeCommand() {
        return new InstantCommand(() -> {
            currentPiece = GamePiece.CUBE;
        });
    }

    private Command handleConeCommand() {
        return new SequentialCommandGroup(
                new WaitUntilCommand(buttonDetectedTrigger),
                new ConditionalCommand(
                        setSpindexerMotor(-0.3).withTimeout(0.3), //TODO: check the minimal time for successful straighten
                        new InstantCommand(),
                        beamBreakDetectedTrigger),
                setSpindexerMotor(0.3),
                new InstantCommand(() -> {
                    currentPiece = GamePiece.CONE;
                }),
                new WaitCommand(0.3),
                new WaitUntilCommand(buttonDetectedTrigger.negate()));
    }

    public boolean isGamePieceDetected() {
        return beamBreakDetectedTrigger.getAsBoolean();
    }


    public GamePiece getCurrentItem() {
        if (colorSensor.getProximity() < DISTANCE_THRESHOLD) {
            if (colorSensor.getBlue() < GAME_PIECE_THRESHOLD) return GamePiece.CONE;
            return GamePiece.CUBE;
        }
        return GamePiece.EMPTY;
    }

    private Command setSpindexerMotor(double speed) {
        return new InstantCommand(() -> spindexer.set(speed), this);
    }
}
