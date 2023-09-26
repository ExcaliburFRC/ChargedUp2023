package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Map;
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

    public int targetPos = MAX_ANGLE_DEGREES;
    public final Trigger isAtTargetPosTrigger = new Trigger(() -> Math.abs(angleEncoder.getDistance() - targetPos) < POS_THRESHOLD).debounce(0.2);

    public Trigger cuberReadyTrigger = isAtTargetVelTrigger.and(isAtTargetPosTrigger);

    public Cuber() {
        angleMotor.restoreFactoryDefaults();
        angleMotor.clearFaults();
        angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        angleMotor.setOpenLoopRampRate(1);
        angleMotor.setInverted(true);

        // motor soft limit
//        angleMotor.setSoftLimit(kForward, FWD_SOFT_LIMIT);
//        angleMotor.enableSoftLimit(kForward, true);
//        angleMotor.setSoftLimit(kReverse, REV_SOFT_LIMIT);
//        angleMotor.enableSoftLimit(kReverse, true);

        shooterMotor.restoreFactoryDefaults();
        shooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        shooterMotor.clearFaults();
        shooterMotor.setInverted(false);

        angleEncoder.reset();
        angleEncoder.setPositionOffset(ABS_ENCODER_OFFSET);
        angleEncoder.setDistancePerRotation(360);

        angleRelativeEncoder.setPositionConversionFactor(ANGLE_CONVERSION_FACTOR);
        angleRelativeEncoder.setPosition(getCuberAngle());

        initShuffleboardData();
        setDefaultCommand(resetCuberCommand());
    }

    public double getCuberAngle(){
        double val = angleEncoder.getDistance();
        val = val < 0? 360 + val : val;;
        return val % 360;
    }

    private boolean isAtFrontLimit(){
        double angle = getCuberAngle();

        return angle >= FWD_SOFT_LIMIT && angle <= FWD_SOFT_LIMIT + 30;
    }

    private boolean isAtReverseLimit(){
        double angle = getCuberAngle();

        return angle <= 360 && angle >= 330;
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
                    double pid = anglePIDcontrller.calculate(getCuberAngle(), angle.angle);
                    double ff = angleFFcontrller.calculate(Math.toRadians(angle.angle), 0);
                    double output = pid + ff / 60.0;

                    if ((isAtFrontLimit() && output > 0) || (isAtReverseLimit() && output < 0)) {
                        angleMotor.stopMotor();
                        DriverStation.reportError("cuber soft limit reached!", false);
                    }
                    else angleMotor.setVoltage(output);
                },
                (__) -> angleMotor.stopMotor(),
                () -> false
        );
    }

    // mainCommands
    private Command requirement() {
        return new RunCommand(() -> {
        }, this);
    }

    public Command resetCuberCommand() {
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
                leds.applyPatternCommand(BLINKING, PURPLE.color),
                new WaitUntilCommand(cuberReadyTrigger).andThen(
                        leds.applyPatternCommand(SOLID, GREEN.color),
                        new WaitUntilCommand(confirm).andThen(pushCubeCommand())),
                requirement())
                .until(hasCubeTrigger.negate().debounce(0.75));
    }

    public Command cannonShooterCommand(DoubleSupplier robotAngle, Trigger override){
        return shootCubeCommand( // TODO: check if PID can get to such high speeds, if not, use open loop.
                SHOOTER_VELOCITIY.CANNON, CUBER_ANGLE.CANNON,
                new Trigger(()-> Math.abs(robotAngle.getAsDouble()) > ROBOT_ANGLE_THRESHOLD).or(override)
        );
    }

    public Command confirmCubeIntake(){
        return new ParallelCommandGroup(
                setCuberAngleCommand(CUBER_ANGLE.IDLE),
                setShooterVelocityCommand(SHOOTER_VELOCITIY.INTAKE),
                requirement()).withTimeout(2);
    }

    public Command toggleIdleModeCommand(){
        return new StartEndCommand(
                ()-> angleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast),
                ()-> angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake))
                .ignoringDisable(true);
    }

    // raw commands
    public Command angleControl(DoubleSupplier speed){
        return new RunCommand(()-> {
            double output = speed.getAsDouble() / 5.0;
            if ((isAtFrontLimit() && output > 0) || (isAtReverseLimit() && output < 0)) angleMotor.stopMotor();
            else angleMotor.set(output);

            System.out.println("front: " + isAtFrontLimit());
            System.out.println("reverse: " + isAtReverseLimit());
        }, this);
    }

    public Command rawIntake(double speed){
        return Commands.runEnd(()-> shooterMotor.set(speed), shooterMotor::stopMotor, this);
    }

    private void initShuffleboardData(){
        cuberTab.addDouble("servo angle", this::getServoAngle).withPosition(8, 0).withSize(4, 4)
                .withWidget("Simple Dial").withProperties(Map.of("min", 0, "max", 90));;
        cuberTab.addDouble("colorProximity", colorSensor::getProximity).withPosition(12, 4).withSize(4, 2)
                .withWidget("Number Slider").withProperties(Map.of("min", 85, "max", 120));
        cuberTab.addDouble("cuber angle", this::getCuberAngle).withPosition(12, 0).withSize(4, 4)
                .withWidget("Simple Dial").withProperties(Map.of("min", 0, "max", 180));

        cuberTab.addBoolean("hasCubeTrigger", hasCubeTrigger).withPosition(8, 4).withSize(4, 2);
        cuberTab.addBoolean("isAtTargetVel", isAtTargetVelTrigger).withPosition(8, 6).withSize(4, 2);
        cuberTab.addBoolean("isAtTargetPos", isAtTargetPosTrigger).withPosition(12, 6).withSize(4, 2);

        cuberTab.addDouble("targetPos", ()-> targetPos).withPosition(13, 8);

        cuberTab.addDouble("velocity", shooterEncoder::getVelocity).withPosition(8, 8);
        cuberTab.addDouble("targetVel", ()-> targetVel).withPosition(10, 8);

        cuberTab.addDouble("applied output", shooterMotor::getAppliedOutput);
        cuberTab.addDouble("get", shooterMotor::getAppliedOutput);

    }
}