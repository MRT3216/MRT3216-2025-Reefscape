package frc.robot.subsystems.Coral.Pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.settings.Constants.CORAL.PIVOT;
import frc.robot.settings.RobotMap.ROBOT.CORAL_SYSTEM.PIVOT_MAP;

public class CoralPivotSubsystem extends SubsystemBase {
    private final SparkFlex motorController;
    private AbsoluteEncoder encoder;
    private ProfiledPIDController pIDController;
    private ArmFeedforward feedforward;
    private boolean enabled = false;
    private CoralPivotSimulation simContainer;

    public CoralPivotSubsystem() {
        motorController = new SparkFlex(PIVOT_MAP.motorCANID, MotorType.kBrushless);

        SparkMaxConfig motorControllerConfig = new SparkMaxConfig();

        motorControllerConfig.idleMode(IdleMode.kBrake)
                .inverted(PIVOT.kMotorInverted)
                .smartCurrentLimit(PIVOT.kMotorCurrentLimit)
                .voltageCompensation(PIVOT.kVoltageCompensation)
                .openLoopRampRate(PIVOT.kPivotRampRate);

        encoder = motorController.getAbsoluteEncoder();
        AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();
        encoderConfig.zeroCentered(true);
        motorControllerConfig.apply(encoderConfig);

        SoftLimitConfig softLimitConfig = new SoftLimitConfig();
        softLimitConfig
                // Soft limits use the internal motor encoder rather than the attached
                // absolute encoder so adjust by the gearing
                .forwardSoftLimit(PIVOT.kSoftForwardLimit.in(Rotations) * PIVOT.kPivotGearing)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(PIVOT.kSoftReverseLimit.in(Rotations) * PIVOT.kPivotGearing)
                .reverseSoftLimitEnabled(true);
        motorControllerConfig.apply(softLimitConfig);

        motorController.configure(motorControllerConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // Set the motor's internal encoder to the absolute position
        motorController.getEncoder().setPosition(encoder.getPosition() * PIVOT.kPivotGearing);

        pIDController = new ProfiledPIDController(
                PIVOT.kPivotKp,
                PIVOT.kPivotKi,
                PIVOT.kPivotKd,
                new TrapezoidProfile.Constraints(
                        PIVOT.kMaxAngularVelocity.in(RotationsPerSecond),
                        PIVOT.kMaxAngularAcceleration.in(RotationsPerSecondPerSecond)));

        feedforward = new ArmFeedforward(
                PIVOT.kPivotkS,
                PIVOT.kPivotkG,
                PIVOT.kPivotkV,
                PIVOT.kPivotkA);

        pIDController.setTolerance(PIVOT.kMaxPivotError.in(Rotations));
        // Set the inital position so that when enabled the controler
        // matches the initial position
        //pIDController.reset(CORAL.POSITIONS.STARTING.getAngle().in(Rotations));
        pIDController.reset(getPivotAngle().in(Rotations));
        if (RobotBase.isSimulation()) {
            this.simContainer = new CoralPivotSimulation(encoder, motorController);
        }
    }

    public Command movePivotToAngle(Angle angle) {
        return this.run(() -> {
            setPivotGoal(angle);
            this.enable();
        }).until(this.atGoal());
    }

    private void setPivotGoal(Angle angle) {
        double goalAngleInRotations = MathUtil.clamp(angle.in(Rotations),
                PIVOT.kMinPivotAngle.in(Rotations),
                PIVOT.kMaxPivotAngle.in(Rotations));

        pIDController.setGoal(goalAngleInRotations);
    }

    protected Trigger atGoal() {
        return new Trigger(() -> pIDController.atGoal());
    }

    public Command movePivot(double speed) {
        return this.startEnd(
                () -> motorController.set(speed),
                () -> motorController.set(0.0));
    }

    public void periodic() {
        if (enabled) {
            double armPidVoltage = pIDController.calculate(getPivotAngle().in(Rotations));
            double ffVoltage = feedforward.calculate(
                    pIDController.getSetpoint().position,
                    pIDController.getSetpoint().velocity);

            motorController.setVoltage(armPidVoltage + ffVoltage);
        }

        SmartDashboard.putBoolean("Coral Pivot Enabled", enabled);
        SmartDashboard.putNumber("Coral Pivot position", getPivotAngle().in(Degrees));
        SmartDashboard.putNumber("Coral Pivot position error",
                Units.rotationsToDegrees(pIDController.getPositionError()));
        SmartDashboard.putNumber("Coral Pivot position setpoint",
                Units.rotationsToDegrees(pIDController.getSetpoint().position));
        SmartDashboard.putNumber("Coral Pivot encoder absolute", encoder.getPosition());
        SmartDashboard.putNumber("Coral Pivot encoder motor", motorController.getEncoder().getPosition());
        SmartDashboard.putNumber("Coral Pivot Motor effort", motorController.getAppliedOutput());
    }

    /** Enables the PID control. Resets the controller. */
    public void enable() {
        enabled = true;
        // This doesn't work
        //pIDController.reset(getPivotAngle().in(Rotations));
    }

    /** Disables the PID control. Sets output to zero. */
    public void disable() {
        enabled = false;
        pIDController.setGoal(getPivotAngle().in(Rotations));
        motorController.set(0);
    }

    private Angle getPivotAngle() {
        return Rotations.of(encoder.getPosition());
    }

    @Override
    public void simulationPeriodic() {
        if (simContainer != null) {
            simContainer.simulationPeriodic();
            SmartDashboard.putBoolean("Coral Pivot Enabled", enabled);
            SmartDashboard.putNumber("Coral Pivot position error",
                    Units.rotationsToDegrees(pIDController.getPositionError()));
            SmartDashboard.putNumber("Coral Pivot position setpoint",
                    Units.rotationsToDegrees(pIDController.getSetpoint().position));
            SmartDashboard.putNumber("Coral Pivot position actual", getPivotAngle().in(Degrees));
            SmartDashboard.putNumber("Coral Pivot encoder", encoder.getPosition());
            SmartDashboard.putNumber("Coral Pivot Motor effort", motorController.getAppliedOutput());
        }
    }
}