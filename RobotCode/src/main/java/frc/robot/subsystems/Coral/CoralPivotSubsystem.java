package frc.robot.subsystems.Coral;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

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
        encoderConfig.positionConversionFactor(360);
        pivotConfig.apply(encoderConfig);

        motorController.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // pivotConfig.softLimit(0, 0);

        pIDController = new ProfiledPIDController(
                CoralPivotConstants.kPivotKp,
                CoralPivotConstants.kPivotKi,
                CoralPivotConstants.kPivotKd,
                new TrapezoidProfile.Constraints(CoralPivotConstants.kMaxAngularVelocity.in(DegreesPerSecond),
                        CoralPivotConstants.kMaxAngularAcceleration.in(DegreesPerSecondPerSecond)));

        feedforward = new ArmFeedforward(
                CoralPivotConstants.kPivotkS,
                CoralPivotConstants.kPivotkG,
                CoralPivotConstants.kPivotkV,
                CoralPivotConstants.kPivotkA);

        pIDController.setTolerance(CoralPivotConstants.kMaxPivotError.in(Degrees));

        if (RobotBase.isSimulation()) {
            this.simContainer = new CoralPivotSimulation(encoder, motorController);
        }
    }

    public Command movePivotToAngle(Angle angle) {
        System.out.println("Angle: " + angle);
        return this.run(() -> {
            this.enable();
            pIDController.setGoal(angle.in(Degrees));
        }).until(this.atGoal());
    }

    protected Trigger atGoal() {
        return new Trigger(() -> pIDController.atGoal());
    }

    public void periodic() {
        if (enabled) {
            double armPidVoltage = pIDController.calculate(getPivotAngle().in(Degrees));
            double ffVoltage = feedforward.calculate(
                    Units.degreesToRadians(pIDController.getSetpoint().position),
                    pIDController.getSetpoint().velocity);

            motorController.setVoltage(armPidVoltage + ffVoltage);

            // TODO: add values
            // double feedforwardOutput = feedforward.calculate(getPivotAngle().in(Degrees),
            //         pIDController.getSetpoint().velocity);
            // double pidOutput = pIDController.calculate(encoder.getPosition(), 0);
            // motorController.setVoltage(feedforwardOutput + pidOutput);
        }
    }

    /** Enables the PID control. Resets the controller. */
    public void enable() {
        enabled = true;
        //pIDController.reset(0); // TODO: add value
    }

    /** Disables the PID control. Sets output to zero. */
    public void disable() {
        enabled = false;
        pIDController.setGoal(getPivotAngle().in(Degrees));
        motorController.set(0);
    }

    private Angle getPivotAngle() {
        return Radians.of(encoder.getPosition());
    }

    @Override
    public void simulationPeriodic() {
        if (simContainer != null) {
            simContainer.simulationPeriodic();
            SmartDashboard.putBoolean("Coral Pivot Enabled", enabled);
            SmartDashboard.putNumber("Coral Pivot position error", pIDController.getPositionError());
            SmartDashboard.putNumber("Coral Pivot position setpoint", pIDController.getSetpoint().position);
            SmartDashboard.putNumber("Coral Pivot position actual", getPivotAngle().in(Degrees));
            SmartDashboard.putNumber("Coral Pivot Motor effort", motorController.getAppliedOutput());
        }
    }
}