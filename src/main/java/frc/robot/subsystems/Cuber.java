package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.CuberConstants.*;
import static frc.robot.subsystems.LEDs.LEDPattern.BLINKING;
import static frc.robot.subsystems.LEDs.LEDPattern.SOLID;
import static frc.robot.utility.Colors.*;


public class Cuber extends SubsystemBase {
    private final CANSparkMax angleMotor = new CANSparkMax(ANGLE_MOTOR_ID, kBrushless);
    private final CANSparkMax shooterMotor = new CANSparkMax(ROLLERS_MOTOR_ID, kBrushless);

    private final Servo servo = new Servo(SERVO_CHANNEL);

    private final RelativeEncoder shooterEncoder = shooterMotor.getEncoder();
    private final RelativeEncoder angleRelativeEncoder = angleMotor.getEncoder();

    private final DutyCycleEncoder angleEncoder = new DutyCycleEncoder(ENCODER_CHANNEL);
    private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    private final LEDs leds = LEDs.getInstance();

    public static final ShuffleboardTab cuberTab = Shuffleboard.getTab("Cuber");

    private final PIDController shooterPIDcontroller = new PIDController(Kp_SHOOTER, 0, Kd_SHOOTER);
    private final SimpleMotorFeedforward shooterFFcontroller = new SimpleMotorFeedforward(Ks_SHOOTER, Kv_SHOOTER, Ka_SHOOTER);

    private final PIDController anglePIDcontrller = new PIDController(Kp_ANGLE, 0, Kd_ANGLE);
    private final ArmFeedforward angleFFcontrller = new ArmFeedforward(Ks_ANGLE, Kg_ANGLE, Kv_ANGLE, Ka_ANGLE);

    public final Trigger hasCubeTrigger = new Trigger(() -> colorSensor.getProximity() >= COLOR_DISTANCE_THRESHOLD).debounce(0.3)
            .onTrue(leds.applyPatternCommand(SOLID, GREEN.color).withTimeout(0.25))
            .onFalse(leds.applyPatternCommand(SOLID, RED.color).withTimeout(0.25));

    public int targetVel = 0;
    public final Trigger isAtTargetVelTrigger = new Trigger(() -> Math.abs(targetVel - shooterEncoder.getVelocity()) < VEL_THRESHOLD).debounce(0.1);

    public int targetPos = 139;
    public final Trigger isAtTargetPosTrigger = new Trigger(() -> Math.abs(angleEncoder.getDistance() - targetPos) < POS_THRESHOLD).debounce(0.2);

    private GenericEntry shooterVel = cuberTab.add("shooterVel", 0).getEntry();

    // high dc: 32
    // mid dc: 23

    public Cuber() {
        angleMotor.restoreFactoryDefaults();
        angleMotor.clearFaults();
        angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

//        angleMotor.setSoftLimit(kForward, FWD_SOFT_LIMIT);
//        angleMotor.enableSoftLimit(kForward, true);
//        angleMotor.setSoftLimit(kReverse, REV_SOFT_LIMIT);
//        angleMotor.enableSoftLimit(kReverse, true);

        angleMotor.setOpenLoopRampRate(3);
        angleMotor.setInverted(false);

        shooterMotor.restoreFactoryDefaults();
        shooterMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shooterMotor.clearFaults();
        shooterMotor.setInverted(false);

        angleEncoder.reset();
        angleEncoder.setPositionOffset(ABS_ENCODER_OFFSET);
        angleEncoder.setDistancePerRotation(360);

        angleRelativeEncoder.setPositionConversionFactor(ANGLE_CONVERSION_FACTOR);
        angleRelativeEncoder.setPosition(angleEncoder.getDistance());

        cuberTab.addDouble("servo angle", this::getServoAngle).withPosition(8, 0).withSize(4, 4)
                .withWidget("Simple Dial").withProperties(Map.of("min", 0, "max", 90));;
        cuberTab.addDouble("colorMM", colorSensor::getProximity).withPosition(12, 4).withSize(4, 2)
                .withWidget("Number Slider").withProperties(Map.of("min", 75, "max", 120));
        cuberTab.addDouble("cuber angle", angleEncoder::getDistance).withPosition(12, 0).withSize(4, 4)
                .withWidget("Simple Dial").withProperties(Map.of("min", 0, "max", 180));

        cuberTab.addBoolean("hasCubeTrigger", hasCubeTrigger).withPosition(8, 4).withSize(4, 2);
        cuberTab.addBoolean("isAtTargetVel", isAtTargetVelTrigger).withPosition(8, 6).withSize(4, 2);
        cuberTab.addBoolean("isAtTargetPos", isAtTargetPosTrigger).withPosition(12, 6).withSize(4, 2);

        setDefaultCommand(closeCuberCommand());
    }

    // Servo Commands
    private Command setServoAngleCommand(SERVO_ANGLE angle) {
        return new RunCommand(() -> servo.setAngle(angle.angle)).until(() -> servo.getAngle() == angle.angle);
    }

    public double getServoAngle() {
        return servo.getAngle();
    }

    private Command retractServoCommand() {
        return setServoAngleCommand(SERVO_ANGLE.RETRACTED);
    }

    private Command pushCubeCommand() {
        return setServoAngleCommand(SERVO_ANGLE.EXTENDED);
    }

    // Shooter Commands
    private Command setShooterVelocityCommand(SHOOTER_VELOCITIY vel) {
        return new FunctionalCommand(
                () -> this.targetVel = vel.velocity,
                () -> {
                    double pid = shooterPIDcontroller.calculate(shooterEncoder.getVelocity(), vel.velocity);
                    double ff = shooterFFcontroller.calculate(vel.velocity, 0) / 60;

                    shooterMotor.setVoltage(pid + ff);

                    SmartDashboard.putNumber("setpoint", vel.velocity);
                    SmartDashboard.putNumber("velocity", shooterEncoder.getVelocity());
                },
                (__) -> {
                    shooterMotor.stopMotor();
                    SmartDashboard.putNumber("setpoint", 0);
                    targetVel = 0;
                },
                () -> false
        );
    }

    private Command stopShooterCommand() {
        return new InstantCommand(shooterMotor::stopMotor);
    }

    // Angle Commands
    private Command setCuberAngleCommand(CUBER_ANGLE angle) {
        return new FunctionalCommand(
                () -> targetPos = angle.angle,
                () -> {
                    double pid = anglePIDcontrller.calculate(angleEncoder.getDistance(), angle.angle);
                    double ff = angleFFcontrller.calculate(Math.toRadians(angle.angle), 0);

                    angleMotor.setVoltage(pid + ff);
                },
                (__) -> {
                },
                () -> false
        );
    }

    // mainCommands
    private Command requirement() {
        return new RunCommand(() -> {
        }, this);
    }

    public Command closeCuberCommand() {
        return new ParallelCommandGroup(
                setCuberAngleCommand(CUBER_ANGLE.IDLE),
                stopShooterCommand(),
                retractServoCommand(),
                requirement());
    }

    public Command intakeCommand(CUBER_ANGLE cuberAngle) {
        return new ParallelCommandGroup(
                setCuberAngleCommand(cuberAngle),
                setShooterVelocityCommand(SHOOTER_VELOCITIY.INTAKE),
                leds.applyPatternCommand(BLINKING, PURPLE.color),
                requirement())
                .until(hasCubeTrigger);
    }

    public Command shootCubeCommand(SHOOTER_VELOCITIY vel, CUBER_ANGLE angle, Trigger confirm) {
        return new ParallelCommandGroup(
                setCuberAngleCommand(angle),
                setShooterVelocityCommand(vel),
                new WaitUntilCommand(isAtTargetVelTrigger.and(isAtTargetPosTrigger).and(confirm))
                        .andThen(pushCubeCommand()),
                leds.applyPatternCommand(BLINKING, PURPLE.color),
                requirement())
                .until(hasCubeTrigger.negate().debounce(0.15));
    }

    public Command toggleIdleModeCommand(){
        return new StartEndCommand(
                ()-> angleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast),
                ()-> angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake))
                .ignoringDisable(true);
    }

    public Command cannonShooterCommand(DoubleSupplier robotAngle, Trigger override){
        return shootCubeCommand(
                SHOOTER_VELOCITIY.CANNON, CUBER_ANGLE.CANNON,
                new Trigger(()-> Math.abs(robotAngle.getAsDouble()) > ROBOT_ANGLE_THRESHOLD).or(override)
        );
    }

    // raw commands
    public Command shootFromShuffleboard(BooleanSupplier pushCube){
        return new RunCommand(()-> shooterMotor.set(shooterVel.getDouble(0) / 100), this)
                .alongWith(new WaitUntilCommand(pushCube).andThen(pushCubeCommand()))
                .until(hasCubeTrigger.negate().debounce(0.15));
    }

    public Command rawIntake(){
        return Commands.runEnd(()-> shooterMotor.set(-0.15), shooterMotor::stopMotor, this)
                .until(hasCubeTrigger);
    }
}