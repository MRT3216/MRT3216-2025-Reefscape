package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.settings.Constants.Coral.ElevatorConstants;
import frc.robot.settings.RobotMap.ROBOT.CoralSystem.ElevatorMap;

public class ElevatorSubsystem extends SubsystemBase {
    // #region Fields

    private final SparkFlex leadMotorController;
    private final SparkFlex followerMotorController;
    private RelativeEncoder encoder;
    private ProfiledPIDController pIDController;
    private ElevatorSimulation elevatorSimContainer;
    private ElevatorFeedforward elevatorFeedForward;
    private boolean enabled;

    // #endregion

    public ElevatorSubsystem() {
        leadMotorController = new SparkFlex(ElevatorMap.leadMotorCANID, MotorType.kBrushless);
        followerMotorController = new SparkFlex(ElevatorMap.followerMotorCANID, MotorType.kBrushless);

        SparkMaxConfig leadConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();

        leadConfig.idleMode(IdleMode.kBrake)
                .inverted(!ElevatorConstants.kLeadMotorInverted)
                .smartCurrentLimit(ElevatorConstants.kMotorCurrentLimit)
                .voltageCompensation(ElevatorConstants.kVoltageCompensation)
                .openLoopRampRate(ElevatorConstants.kElevatorRampRate);

        // TODO: This
        // EncoderConfig encoderConfig = new EncoderConfig();
        // // Convert rotations to degrees
        // //encoderConfig.positionConversionFactor(360);
        // leadConfig.apply(encoderConfig);

        leadMotorController.configure(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        followerConfig.follow(leadMotorController, ElevatorConstants.kLeadMotorInverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(ElevatorConstants.kMotorCurrentLimit)
                .voltageCompensation(ElevatorConstants.kVoltageCompensation);

        followerMotorController.configure(followerConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        //TODO: replace relative encoder with absolute encoder
        // NOOOO! Use the absolute encoder to set the zero position of the elvator
        // encoder = leadMotor.getAbsoluteEncoder();
        // AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();
        // encoderConfig.positionConversionFactor(360);
        encoder = leadMotorController.getEncoder();
        encoder.setPosition(0);

        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
                ElevatorConstants.kMaxElevatorVelocity.in(MetersPerSecond),
                ElevatorConstants.kMaxElevatorAcceleration.in(MetersPerSecondPerSecond));

        pIDController = new ProfiledPIDController(ElevatorConstants.kElevatorKp, ElevatorConstants.kElevatorKi,
                ElevatorConstants.kElevatorKd, constraints);

        elevatorFeedForward = new ElevatorFeedforward(ElevatorConstants.kElevatorkS,
                ElevatorConstants.kElevatorkG, ElevatorConstants.kElevatorkV, ElevatorConstants.kElevatorkA);

        pIDController.setTolerance(ElevatorConstants.kPositionTolerance.in(Meters));

        enabled = false;

        if (RobotBase.isSimulation()) {
            this.elevatorSimContainer = new ElevatorSimulation(encoder, leadMotorController);
        }
    }

    public Command moveElevatorToHeight(Distance height) {
        return this.run(() -> {
            this.enable();
            setElevatorHeightGoal(height);
        }).until(this.atGoal());
    }

    private void setElevatorHeightGoal(Distance height) {
        double goalHeightinMeters = MathUtil.clamp(height.in(Meters),
                ElevatorConstants.kMinHeight.in(Meters),
                ElevatorConstants.kMaxHeight.in(Meters));
        pIDController.setGoal(goalHeightinMeters);
    }

    protected Trigger atGoal() {
        return new Trigger(() -> pIDController.atGoal());
    }

    /** Enables the PID control. Resets the controller. */
    protected void enable() {
        enabled = true;
    }

    /** Disables the PID control. Sets output to zero. */
    public void disable() {
        pIDController.setGoal(encoder.getPosition());
        leadMotorController.set(0);
        enabled = false;
    }

    public void periodic() {
        if (enabled) {
            double pidOutput = pIDController.calculate(encoder.getPosition());
            double feedForward = elevatorFeedForward.calculate(pIDController.getSetpoint().velocity);
            double motorEffortVoltage = pidOutput + feedForward;

            leadMotorController.setVoltage(motorEffortVoltage);
        }
    }

    @Override
    public void simulationPeriodic() {
        if (elevatorSimContainer != null) {
            elevatorSimContainer.simulationPeriodic();
            SmartDashboard.putBoolean("Elevator Enabled", enabled);
            SmartDashboard.putNumber("Elevator position error", pIDController.getPositionError());
            SmartDashboard.putNumber("Elevator position setpoint", pIDController.getSetpoint().position);
            SmartDashboard.putNumber("Elevator position actual", encoder.getPosition());
            SmartDashboard.putNumber("Elevator Motor effort", leadMotorController.getAppliedOutput());
        }
    }
}