package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static edu.wpi.first.wpilibj.PneumaticsModuleType.REVPH;
import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor = new CANSparkMax(k_INTAKE_MOTOR_ID, kBrushless);

  private final DoubleSolenoid piston = new DoubleSolenoid(REVPH, k_FWD_CHANNEL, k_REV_CHANNEL);

  public Intake() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setSmartCurrentLimit(k_INTAKE_MOTOR_CURRENT_LIMIT);
    intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    intakeMotor.setInverted(false); //TODO: check
  }

  public Command manualCommand(DoubleSupplier intakeSpeed, BooleanSupplier togglePiston) {
    return new RunCommand(
          () -> {
            intakeMotor.set(intakeSpeed.getAsDouble());

            if (togglePiston.getAsBoolean())
              if (piston.get().equals(DoubleSolenoid.Value.kReverse)) piston.set(DoubleSolenoid.Value.kForward);
              else piston.set(DoubleSolenoid.Value.kReverse);
          },
          this);
  }

  public Command openPiston() {
    return new RunCommand(() -> {
      piston.set(DoubleSolenoid.Value.kForward);
    }, this);
  }

  public Command closePiston() {
    return new RunCommand(() -> {
      piston.set(DoubleSolenoid.Value.kReverse);
    },this);
  }

  public Command openIntake() {
    return new RunCommand(() -> {
      intakeMotor.set(0.5);
      openPiston();
    }, this);
  }

  public Command closeIntake() {
    return new RunCommand(() -> {
      intakeMotor.set(0);
      closePiston();
    }, this);
  }

  public Command toggleIntakeCommand(){
    return new FunctionalCommand(
          this::openPiston,
          ()-> intakeMotor.set(0.3),
          (__) -> closePiston(),
          ()-> false);
  }
}
