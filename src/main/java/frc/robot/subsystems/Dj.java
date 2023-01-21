package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.IntakeConstants.*;

public class Dj extends SubsystemBase {

  private final CANSparkMax DJMotor = new CANSparkMax(k_DJ_MOTOR_ID, kBrushless);
  private final DigitalInput button = new DigitalInput(BUTTON_CHANNEL);
  private final DigitalInput beamBreaker = new DigitalInput(BEAMBREAK_CHANNEL);
  private final ColorSensorV3 colorSensorV3 = new ColorSensorV3(I2C.Port.kMXP);

  private final Trigger beamBreakDetected = new Trigger(beamBreaker::get);
  //when beam brakers are connected its
  private final Trigger buttonDetected = new Trigger(button::get);
  //when button pressed its false

  public Dj() {
    DJMotor.restoreFactoryDefaults();
    DJMotor.setSmartCurrentLimit(k_DJ_MOTOR_CURRENT_LIMIT);
    DJMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    DJMotor.setInverted(false); //TODO: check
  }

  public Command straightenConeCommand() {
    return new SequentialCommandGroup(
          setDjMotor(0.3)
          .until(buttonDetected),
          new ConditionalCommand(
                setDjMotor(-0.3).raceWith(new WaitCommand(0.5)),
                new InstantCommand(),
                beamBreakDetected),
          new WaitUntilCommand(buttonDetected.negate())
    ).repeatedly();
  }

  public Command setDjMotor(double speed) {
    return new RunCommand(() -> DJMotor.set(speed), this);
  }
}
