package frc.robot.subsystems.Coral.Elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.settings.Constants.CORAL;
import frc.robot.settings.Constants.CORAL.ELEVATOR;
import frc.robot.settings.Constants.CORAL.POSITIONS;
import frc.robot.settings.RobotMap.ROBOT.CORAL_SYSTEM.ELEVATOR_MAP;

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
        leadMotorController = new SparkFlex(ELEVATOR_MAP.leadMotorCANID, MotorType.kBrushless);
        followerMotorController = new SparkFlex(ELEVATOR_MAP.followerMotorCANID, MotorType.kBrushless);

        SparkMaxConfig leadConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();

        leadConfig.idleMode(IdleMode.kBrake)
                .inverted(ELEVATOR.kLeadMotorInverted)
                .smartCurrentLimit(ELEVATOR.kMotorCurrentLimit)
                .voltageCompensation(ELEVATOR.kVoltageCompensation)
                .openLoopRampRate(ELEVATOR.kElevatorRampRate);

        encoder = leadMotorController.getEncoder();
        EncoderConfig encoderConfig = new EncoderConfig();
        encoderConfig.positionConversionFactor(1);
        leadConfig.apply(encoderConfig);
        encoder.setPosition(0);

        // SoftLimitConfig softLimitConfig = new SoftLimitConfig();
        // softLimitConfig
        //         // Soft limits use the internal motor encoder rather than the attached
        //         // absolute encoder so adjust by the gearing
        //         .forwardSoftLimit(ELEVATOR.kSoftMaxHeight.in(Meters))
        //         .forwardSoftLimitEnabled(true)
        //         .reverseSoftLimit(ELEVATOR.kSoftMinHeight.in(Meters))
        //         .reverseSoftLimitEnabled(true);
        // leadConfig.apply(softLimitConfig);

        leadMotorController.configure(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // The follower motor should rotate opposite the lead motor
        // for this gearbox
        followerConfig.follow(leadMotorController, true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(ELEVATOR.kMotorCurrentLimit)
                .voltageCompensation(ELEVATOR.kVoltageCompensation);

        followerMotorController.configure(followerConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
                ELEVATOR.kMaxElevatorVelocity.in(MetersPerSecond),
                ELEVATOR.kMaxElevatorAcceleration.in(MetersPerSecondPerSecond));

        pIDController = new ProfiledPIDController(ELEVATOR.kElevatorKp, ELEVATOR.kElevatorKi,
                ELEVATOR.kElevatorKd, constraints);

        elevatorFeedForward = new ElevatorFeedforward(ELEVATOR.kElevatorkS,
                ELEVATOR.kElevatorkG, ELEVATOR.kElevatorkV, ELEVATOR.kElevatorkA);

        pIDController.setTolerance(ELEVATOR.kPositionTolerance.in(Meters));
        // Set the inital position so that when enabled the controler
        // matches the initial position
        //pIDController.reset(this.targetPosition.getHeight().in(Meters));
        pIDController.reset(this.getPositionDistance().in(Meters));
        enabled = false;

        if (RobotBase.isSimulation()) {
            this.elevatorSimContainer = new ElevatorSimulation(leadMotorController,
                    () -> getPositionDistance().in(Meters));
        }
    }

    public void periodic() {
        if (enabled) {
            double pidOutput = pIDController.calculate(getPositionDistance().in(Meters));
            double feedForward = elevatorFeedForward.calculateWithVelocities(
                    getVelocity().in(MetersPerSecond), pIDController.getSetpoint().velocity);
            double motorEffortVoltage = pidOutput + feedForward;

            leadMotorController.setVoltage(motorEffortVoltage);
        }

        SmartDashboard.putBoolean("Elevator Enabled", enabled);
        SmartDashboard.putNumber("Elevator position", getPositionDistance().in(Meters));
        SmartDashboard.putNumber("Elevator pos. error inches", Units.metersToInches(pIDController.getPositionError()));
        SmartDashboard.putNumber("Elevator position setpoint", pIDController.getSetpoint().position);
        SmartDashboard.putNumber("Elevator position goal", pIDController.getGoal().position);
        SmartDashboard.putNumber("Elevator encoder", encoder.getPosition());
        SmartDashboard.putNumber("Elevator Motor effort", leadMotorController.getAppliedOutput());
        SmartDashboard.putNumber("Elevator Motor effort", leadMotorController.getAppliedOutput());
    }

    private void setElevatorHeightGoal(Distance height) {
        double goalHeightinMeters = MathUtil.clamp(height.in(Meters),
                ELEVATOR.kMinHeight.in(Meters),
                ELEVATOR.kMaxHeight.in(Meters));
        pIDController.setGoal(goalHeightinMeters);
    }

    private Distance getPositionDistance() {
        return Meters.of(encoder.getPosition() * (2 * Math.PI * ELEVATOR.kElevatorDrumRadius)
                / ELEVATOR.kElevatorGearing);
    }

    private LinearVelocity getVelocity() {
        return MetersPerSecond.of(
                (encoder.getVelocity() / 60) * (2 * Math.PI * ELEVATOR.kElevatorDrumRadius)
                        / ELEVATOR.kElevatorGearing);
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

    // #region Commands and Triggers

    // public Command moveElevatorToHeight(POSITIONS position) {
    //     return this.run(() -> {
    //         this.enable();
    //         setElevatorHeightGoal(position.getHeight());
    //     }).until(this.nearGoal());
    // }

    public Command moveElevatorToPosition(POSITIONS position) {
        return this.runOnce(() -> {
            setElevatorHeightGoal(position.getHeight());
            this.enable();
        });
    }

    public Command adjustElevatorHeight(Distance heightAdjustment) {
        return Commands.runOnce(
                () -> {
                    enable();
                    setElevatorHeightGoal(
                            Meters.of(pIDController.getGoal().position).plus(heightAdjustment));
                });
    }

    public Trigger aboveHeight() {
        return new Trigger(() -> getPositionDistance().gt(Meters.of(ELEVATOR.slowModeHeight)));
    }

    public Trigger atGoal() {
        return new Trigger(() -> pIDController.atGoal());
    }

    public Trigger approachingPosition(POSITIONS pos) {
        return new Trigger(
                () -> MathUtil.isNear(pos.getHeight().in(Inches),
                        this.getPositionDistance().in(Inches), ELEVATOR.kNearTargetHeight.in(Inches)));
    }

    // #endregion

    @Override
    public void simulationPeriodic() {
        if (elevatorSimContainer != null) {
            elevatorSimContainer.simulationPeriodic();
        }
    }
}