package frc.robot.subsystems.Algae.Pivot;

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.settings.Constants.ALGAE.PIVOT;
import frc.robot.settings.Constants.ALGAE.PIVOT.Positions;
import frc.robot.settings.RobotMap.ROBOT.ALGAE_SYSTEM.PIVOT_MAP;

public class AlgaePivotSubsystem extends SubsystemBase {
    private final SparkFlex motorController;
    private AbsoluteEncoder absoluteEncoder;
    private ProfiledPIDController pIDController;
    private ArmFeedforward feedforward;
    private boolean enabled = false;
    private AlgaePivotSimulation simContainer;

    public AlgaePivotSubsystem() {
        motorController = new SparkFlex(PIVOT_MAP.motorCANID, MotorType.kBrushless);

        SparkMaxConfig motorControllerConfig = new SparkMaxConfig();

        motorControllerConfig.idleMode(IdleMode.kBrake)
                .inverted(PIVOT.kMotorInverted)
                .smartCurrentLimit(PIVOT.kMotorCurrentLimit)
                .voltageCompensation(PIVOT.kVoltageCompensation)
                .openLoopRampRate(PIVOT.kPivotRampRate);

        absoluteEncoder = motorController.getAbsoluteEncoder();
        AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();
        encoderConfig.zeroCentered(true);
        motorControllerConfig.apply(encoderConfig);

        // SoftLimitConfig softLimitConfig = new SoftLimitConfig();
        // softLimitConfig
        //         // Soft limits use the internal motor encoder rather than the attached
        //         // absolute encoder so adjust by the gearing
        //         .forwardSoftLimit(PIVOT.kSoftForwardLimit.in(Rotations) * PIVOT.kPivotGearing)
        //         .forwardSoftLimitEnabled(true)
        //         .reverseSoftLimit(PIVOT.kSoftReverseLimit.in(Rotations) * PIVOT.kPivotGearing)
        //         .reverseSoftLimitEnabled(true);
        // motorControllerConfig.apply(softLimitConfig);

        motorController.configure(motorControllerConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // Set the motor's internal encoder to the absolute position
        motorController.getEncoder().setPosition(absoluteEncoder.getPosition() * PIVOT.kPivotGearing);

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
        pIDController.reset(getPivotAngle().in(Rotations));

        if (RobotBase.isSimulation()) {
            this.simContainer = new AlgaePivotSimulation(absoluteEncoder, motorController);
        }
    }

    public void periodic() {
        if (enabled) {
            double armPidVoltage = pIDController.calculate(getPivotAngle().in(Rotations));
            double ffVoltage = feedforward.calculate(
                    pIDController.getSetpoint().position,
                    pIDController.getSetpoint().velocity);

            motorController.setVoltage(armPidVoltage + ffVoltage);
        }

        SmartDashboard.putBoolean("Algae Pivot Enabled", enabled);
        SmartDashboard.putNumber("Algae Pivot position", getPivotAngle().in(Degrees));
        SmartDashboard.putNumber("Algae Pivot position error",
                Units.rotationsToDegrees(pIDController.getPositionError()));
        SmartDashboard.putNumber("Algae Pivot position setpoint",
                Units.rotationsToDegrees(pIDController.getSetpoint().position));
        SmartDashboard.putNumber("Algae Pivot position goal",
                Units.rotationsToDegrees(pIDController.getGoal().position));
        // SmartDashboard.putNumber("Algae Pivot encoder absolute", absoluteEncoder.getPosition());
        // SmartDashboard.putNumber("Algae Pivot encoder motor", motorController.getEncoder().getPosition());
        // SmartDashboard.putNumber("Algae Pivot Motor effort", motorController.getAppliedOutput());
    }

    private void setPivotGoal(Angle angle) {
        double goalAngleInRotations = MathUtil.clamp(angle.in(Rotations),
                PIVOT.kMinPivotAngle.in(Rotations),
                PIVOT.kMaxPivotAngle.in(Rotations));
        pIDController.setGoal(goalAngleInRotations);
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

    public Command movePivot(double speed) {
        return this.startEnd(
                () -> motorController.set(speed),
                () -> motorController.set(0.0));
    }

    private Angle getPivotAngle() {
        return Rotations.of(absoluteEncoder.getPosition());
    }

    // #region Commands and Triggers

    public Command movePivotToAngle(Positions angle) {
        return this.run(() -> {
            setPivotGoal(angle.getAngle());
            this.enable();
        }).until(this.atGoal());
    }

    public Command adjustPivotAngle(Angle angleAdjustment) {
        return Commands.runOnce(
                () -> {
                    enable();
                    setPivotGoal(
                            Degrees.of(pIDController.getGoal().position + angleAdjustment.in(Rotations)));
                });
    }

    protected Trigger atGoal() {
        return new Trigger(() -> pIDController.atGoal());
    }

    // #endregion

    @Override
    public void simulationPeriodic() {
        if (simContainer != null) {
            simContainer.simulationPeriodic();
            //motorEncoder.setPosition(absoluteEncoder.getPosition() * PIVOT.kPivotGearing);
            // SmartDashboard.putBoolean("Algae Pivot Enabled", enabled);
            // SmartDashboard.putNumber("Algae Pivot position error",
            //         Units.rotationsToDegrees(pIDController.getPositionError()));
            // SmartDashboard.putNumber("Algae Pivot position setpoint",
            //         Units.rotationsToDegrees(pIDController.getSetpoint().position));
            // SmartDashboard.putNumber("Algae Pivot position actual", getPivotAngle().in(Rotations));
            // SmartDashboard.putNumber("Algae Pivot Motor effort", motorController.getAppliedOutput());
        }
    }
}