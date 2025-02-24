package frc.robot.subsystems.Coral;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
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
import frc.robot.settings.Constants.CoralPivotConstants;
import frc.robot.settings.RobotMap.ROBOT.PivotMap;

public class CoralPivotSubsystem extends SubsystemBase {
    private final SparkFlex motorController;
    private AbsoluteEncoder encoder;
    private ProfiledPIDController pIDController;
    private ArmFeedforward feedforward;
    private boolean enabled;
    private CoralPivotSimulation simContainer;

    public CoralPivotSubsystem() {
        motorController = new SparkFlex(PivotMap.pivotMotorCANID, MotorType.kBrushless);

        SparkMaxConfig pivotConfig = new SparkMaxConfig();

        pivotConfig.idleMode(IdleMode.kBrake)
                .inverted(CoralPivotConstants.kMotorInverted)
                .smartCurrentLimit(CoralPivotConstants.kMotorCurrentLimit)
                .voltageCompensation(CoralPivotConstants.kVoltageCompensation)
                .openLoopRampRate(CoralPivotConstants.kPivotRampRate);

        encoder = motorController.getAbsoluteEncoder();
        EncoderConfig encoderConfig = new EncoderConfig();
        pivotConfig.apply(encoderConfig);

        motorController.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pIDController = new ProfiledPIDController(
                CoralPivotConstants.kPivotKp,
                CoralPivotConstants.kPivotKi,
                CoralPivotConstants.kPivotKd,
                new TrapezoidProfile.Constraints(
                        Units.radiansPerSecondToRotationsPerMinute(
                                CoralPivotConstants.kMaxAngularVelocity.in(RotationsPerSecond)),
                        Units.radiansPerSecondToRotationsPerMinute(
                                CoralPivotConstants.kMaxAngularAcceleration.in(RotationsPerSecondPerSecond))));

        feedforward = new ArmFeedforward(
                CoralPivotConstants.kPivotkS,
                CoralPivotConstants.kPivotkG,
                CoralPivotConstants.kPivotkV,
                CoralPivotConstants.kPivotkA);

        pIDController.setTolerance(CoralPivotConstants.kMaxPivotError.in(Rotations));
        // Set the inital position so that when enabled the controler
        // matches the initial position
        pIDController.reset(CoralPivotConstants.kStartingAngle.in(Rotations));

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
                CoralPivotConstants.kMinPivotAngle.in(Rotations),
                CoralPivotConstants.kMaxPivotAngle.in(Rotations));
        pIDController.setGoal(goalAngleInRotations);
        SmartDashboard.putNumber("Coral Pivot GOAL",
                goalAngleInRotations);
    }

    protected Trigger atGoal() {
        return new Trigger(() -> pIDController.atGoal());
    }

    public void periodic() {
        if (enabled) {
            double armPidVoltage = pIDController.calculate(getPivotAngle().in(Rotations));
            double ffVoltage = feedforward.calculate(
                    pIDController.getSetpoint().position,
                    pIDController.getSetpoint().velocity);

            motorController.setVoltage(armPidVoltage + ffVoltage);
        }
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
            SmartDashboard.putNumber("Coral Pivot Motor effort", motorController.getAppliedOutput());
        }
    }
}