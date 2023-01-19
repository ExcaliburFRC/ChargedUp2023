package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static edu.wpi.first.wpilibj.PneumaticsModuleType.REVPH;
import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor = new CANSparkMax(k_INTAKE_MOTOR_ID, kBrushless);
  private final CANSparkMax DJMotor = new CANSparkMax(k_DJ_MOTOR_ID, kBrushless);

  private final DoubleSolenoid piston = new DoubleSolenoid(REVPH, k_FWD_CHANNEL, k_REV_CHANNEL);

  public Intake() {
    intakeMotor.restoreFactoryDefaults();
    DJMotor.restoreFactoryDefaults();

    intakeMotor.setSmartCurrentLimit(k_INTAKE_MOTOR_CURRENT_LIMIT);
    DJMotor.setSmartCurrentLimit(k_DJ_MOTOR_CURRENT_LIMIT);

    intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    DJMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    intakeMotor.setInverted(false); //TODO: check
    DJMotor.setInverted(false); //TODO: check
  }

  public Command manualCommand(DoubleSupplier intakeSpeed, DoubleSupplier DJSpeed, BooleanSupplier togglePiston) {
    return new RunCommand(
          () -> {
            intakeMotor.set(intakeSpeed.getAsDouble());
            DJMotor.set(DJSpeed.getAsDouble());

            if (togglePiston.getAsBoolean())
              if (piston.get().equals(DoubleSolenoid.Value.kReverse)) piston.set(DoubleSolenoid.Value.kForward);
              else piston.set(DoubleSolenoid.Value.kReverse);
          },
          this);
  }
}
