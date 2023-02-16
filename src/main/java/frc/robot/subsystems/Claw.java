package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

import java.util.PrimitiveIterator;

import static frc.robot.Constants.ClawConstants.*;
import static frc.robot.Constants.IntakeConstants.DISTANCE_THRESHOLD;
import static frc.robot.Constants.IntakeConstants.GAME_PIECE_THRESHOLD;

public class Claw extends SubsystemBase {
  private final DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.REVPH, FORWARD_CHANNEL, REVERSE_CHANNEL);

  private final DigitalInput beambreak = new DigitalInput(BEAMBREAK_CHANNEL);
  private final DigitalInput button = new DigitalInput(BUTTON_CHANNEL);
private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kMXP);
  private final Trigger isClawOpenedTrigger = new Trigger(() -> piston.get().equals(DoubleSolenoid.Value.kReverse));
  private final Trigger beambreakDetectedTrigger = new Trigger(() -> !beambreak.get());
  private final Trigger clawFullTrigger = new Trigger(() -> !button.get());
//  private final Trigger ColorSensorTrigger = new Trigger(()-> !colorSensor.get());

  public Claw() {
  }


  public GamePiece getCurrentItem() {
    if (colorSensor.getProximity() < DISTANCE_THRESHOLD) {
      if (colorSensor.getBlue() < GAME_PIECE_THRESHOLD) return GamePiece.CONE;
      return GamePiece.CUBE;
    }
    return GamePiece.EMPTY;
  }

  public Command autoClawCommand() {
    return new ConditionalCommand(
            autoCloseCommand(),
            openClawCommand(),
            clawFullTrigger);
  }

  private RunCommand autoCloseCommand() {
    return new RunCommand(
            () -> {
              if (beambreakDetectedTrigger.getAsBoolean() && isClawOpenedTrigger.getAsBoolean())
                piston.set(DoubleSolenoid.Value.kForward);
            });
  }

  public Command openClawCommand() {
    return new InstantCommand(() -> piston.set(DoubleSolenoid.Value.kReverse), this);
  }

  private Command closeClawCommand() {
    return new InstantCommand(() -> piston.set(DoubleSolenoid.Value.kForward), this);
  }

  public Command togglePistonCommand() {
    return new ConditionalCommand(
            new InstantCommand(() -> piston.set(DoubleSolenoid.Value.kForward), this),
            new InstantCommand(piston::toggle, this),
            () -> piston.get().equals(DoubleSolenoid.Value.kOff));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("claw full", clawFullTrigger, null);
    builder.addBooleanProperty("claw beam-break", beambreakDetectedTrigger, null);
  }
}
