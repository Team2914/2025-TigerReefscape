package com.team2914.robot.subsystems.drivetrain;

import com.team2914.robot.Constants.ModuleConstants;
import com.team2914.robot.subsystems.drivetrain.simulation.SwerveModuleSim;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.apriltag.AprilTagDetector.Config;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class SwerveModule {
    private final SparkMax drivingSparkMax;
    private final SparkMax turningSparkMax;

    private final SparkMaxConfig drivingSparkMaxConfig;
    private final SparkMaxConfig turningSparkMaxConfig;

    private final RelativeEncoder drivingEncoder;
    private final AbsoluteEncoder turningEncoder;

    private final SparkClosedLoopController drivingClosedLoopController;
    private final SparkClosedLoopController turningClosedLoopController;

    private double chassisAngularOffset = 0;
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    private SwerveModuleSim simModule;

    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
        drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
        turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);

        drivingSparkMaxConfig = new SparkMaxConfig();
        turningSparkMaxConfig = new SparkMaxConfig();
        
        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        drivingSparkMax.configure(drivingSparkMaxConfig, null, null);
        turningSparkMax.configure(turningSparkMaxConfig, null, null);

        // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        drivingEncoder = drivingSparkMax.getEncoder();
        turningEncoder = turningSparkMax.getAbsoluteEncoder();
        drivingClosedLoopController = drivingSparkMax.getClosedLoopController();
        turningClosedLoopController = turningSparkMax.getClosedLoopController();

        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.

        drivingSparkMaxConfig.encoder
            .positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
            .velocityConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        turningSparkMaxConfig.encoder
            .positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
            .velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        turningSparkMaxConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        //turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.

        turningSparkMaxConfig.closedLoop
            .positionWrappingEnabled(true)
            .positionWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput)
            .positionWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

        // Set the PID gains for the driving motor. Note these are example gains, and you
        // may need to tune them for your own robot!

        drivingSparkMaxConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD)
            .velocityFF(ModuleConstants.kDrivingFF)
            .outputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);

        // Set the PID gains for the turning motor.

        turningSparkMaxConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD)
            .velocityFF(ModuleConstants.kTurningFF)
            .outputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput);;

        drivingSparkMaxConfig
            .idleMode(ModuleConstants.kDrivingMotorIdleMode)
            .smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);

        turningSparkMaxConfig
            .idleMode(ModuleConstants.kTurningMotorIdleMode)
            .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.

        drivingSparkMax.configure(drivingSparkMaxConfig, null, PersistMode.kPersistParameters);
        turningSparkMax.configure(turningSparkMaxConfig, null, PersistMode.kPersistParameters);

        this.chassisAngularOffset = chassisAngularOffset;
        desiredState.angle = new Rotation2d(turningEncoder.getPosition());
        drivingEncoder.setPosition(0);

        if (RobotBase.isSimulation()) {
            simModule = new SwerveModuleSim();
        }
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        if (RobotBase.isSimulation()) {
            return simModule.getState();
        }
        
        return new SwerveModuleState(drivingEncoder.getVelocity(),
                new Rotation2d(turningEncoder.getPosition() - this.chassisAngularOffset));
    }

    public void setClosedLoopRampRate(double rate){
        drivingSparkMaxConfig.closedLoopRampRate(rate);
        turningSparkMaxConfig.closedLoopRampRate(rate);

        drivingSparkMax.configure(drivingSparkMaxConfig, null, PersistMode.kPersistParameters);
        turningSparkMax.configure(turningSparkMaxConfig, null, PersistMode.kPersistParameters);
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        if (RobotBase.isSimulation()) {
            return simModule.getPosition();
        }

        return new SwerveModulePosition(
                drivingEncoder.getPosition(),
                new Rotation2d(turningEncoder.getPosition() - this.chassisAngularOffset));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(this.chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                new Rotation2d(turningEncoder.getPosition()));

        // Command driving and turning SPARKS MAX towards their respective setpoints.


        //System.out.println("Optimized: (drive) " + optimizedDesiredState.speedMetersPerSecond + "; (turn)" + optimizedDesiredState.angle.getRadians());
        //System.out.println("(driveEncoder) " + drivingEncoder.getPosition() + " & " + drivingEncoder.getVelocity());
        System.out.println("TurnEncoder position is " + turningEncoder.getPosition() + " & encoding velocity is " + turningEncoder.getVelocity());
        //System.out.prin tln("Log: " + optimizedDesiredState.speedMetersPerSecond + " " + optimizedDesiredState.angle.getRadians());
        drivingClosedLoopController.setReference(optimizedDesiredState.speedMetersPerSecond,
                SparkMax.ControlType.kVelocity);
        turningClosedLoopController.setReference(optimizedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);

        this.desiredState = desiredState;

        if (RobotBase.isSimulation()) {
            simModule.updateState(this.desiredState);
        }
        
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        drivingEncoder.setPosition(0);
    }

    public void reverseMotor() {
        drivingSparkMax.setInverted(true);
    }
}
