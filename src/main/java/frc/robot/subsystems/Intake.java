package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.Map;

public class Intake extends SubsystemBase {
    private final DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,0,0);
    private final CANSparkMax intakeMotor = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
    public Intake(){
        intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        intakeMotor.setInverted(false);
    }
    public Command openPistonCommand(){
        return new InstantCommand(()-> piston.set(DoubleSolenoid.Value.kForward));
    }
}
