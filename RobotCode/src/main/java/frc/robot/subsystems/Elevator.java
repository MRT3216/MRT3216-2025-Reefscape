package frc.robot.subsystems;

import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private static Elevator instance;
    private final SparkFlex leadMotor;
    private final SparkFlex followerMotor;
    private SparkAbsoluteEncoder encoder;
    private ProfiledPIDController pidController;

    // TODO: replace with actual values
    private Elevator() {
        int LEADdeleteMe = 11;
        int FOLLOWdeleteMe = 12;
        leadMotor = new SparkFlex(LEADdeleteMe, MotorType.kBrushless);
        followerMotor = new SparkFlex(FOLLOWdeleteMe, MotorType.kBrushless);

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        SparkMaxConfig leadConfig = new SparkMaxConfig();

        followerConfig.follow(LEADdeleteMe, true) // (deviceID, invert)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(FOLLOWdeleteMe)
                .voltageCompensation(FOLLOWdeleteMe);
        followerMotor.configure(followerConfig, null, null); // TODO: improve this bs

        leadConfig.idleMode(IdleMode.kBrake)
                .smartCurrentLimit(LEADdeleteMe)
                .voltageCompensation(LEADdeleteMe);
        leadMotor.configure(leadConfig, null, null);

        encoder = leadMotor.getAbsoluteEncoder();
        AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();
        encoderConfig.positionConversionFactor(360);
        // how to configure the encoder to the encoderConfig?

        pidController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0)); // replace with
                                                                                                    // actual values
    }

    public void setGoal(double inches) {
        // Set the elevator to a certain position in inches
    }
}

//