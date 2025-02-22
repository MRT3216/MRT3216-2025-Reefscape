package frc.robot.subsystems.Coral;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

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
import frc.robot.settings.Constants.CoralPivotConstants;
import frc.robot.settings.RobotMap.ROBOT.PivotMap;

public class CoralPivotSubsystem {
    private final SparkFlex pivotMotor;
    private AbsoluteEncoder pivotEncoder;
    private ProfiledPIDController pIDController;
    private ArmFeedforward feedforward;
    private boolean enabled;

    public CoralPivotSubsystem() {
        pivotMotor = new SparkFlex(PivotMap.pivotMotorCANID, MotorType.kBrushless);

        SparkMaxConfig pivotConfig = new SparkMaxConfig();

        pivotConfig.idleMode(IdleMode.kBrake)
                .inverted(CoralPivotConstants.kMotorInverted)
                .smartCurrentLimit(CoralPivotConstants.kMotorCurrentLimit)
                .voltageCompensation(CoralPivotConstants.kVoltageCompensation)
                .openLoopRampRate(CoralPivotConstants.kPivotRampRate);

        pivotEncoder = pivotMotor.getAbsoluteEncoder();
        EncoderConfig encoderConfig = new EncoderConfig();
        encoderConfig.positionConversionFactor(360);
        pivotConfig.apply(encoderConfig);

        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // pivotConfig.softLimit(0, 0);

        pIDController = new ProfiledPIDController(
                CoralPivotConstants.kPivotKp,
                CoralPivotConstants.kPivotKi,
                CoralPivotConstants.kPivotKd,
                new TrapezoidProfile.Constraints(CoralPivotConstants.kMaxAngularVelocity.in(RadiansPerSecond),
                        CoralPivotConstants.kMaxAngularAcceleration.in(RadiansPerSecondPerSecond)));

        feedforward = new ArmFeedforward(
                CoralPivotConstants.kPivotkS,
                CoralPivotConstants.kPivotkG,
                CoralPivotConstants.kPivotkV,
                CoralPivotConstants.kPivotkA);
    }

    public void periodic() {
        if (enabled) {
            // TODO: add values
            double feedforwardOutput = 0;
            double pidOutput = pIDController.calculate(pivotEncoder.getPosition(), 0);
            pivotMotor.setVoltage(feedforwardOutput + pidOutput);
        }
    }

    public void enable() {
        enabled = true;
        pIDController.reset(0); // TODO: add value
    }

    public void disable() {
        enabled = false;
        pivotMotor.set(0);
    }

    private double getPivotDegrees() {
        return pivotEncoder.getPosition() - CoralPivotConstants.kOffset.in(Degrees);
    }
}