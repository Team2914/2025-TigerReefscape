package com.team2914.robot.subsystems.simulations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

// Kudos to YAGSL
public class SwerveModuleSim {
    private double dt;
    private double prevTime;
    private SwerveModuleState state;
    private double pos;
    private double speed;

    public SwerveModuleSim() {
        state = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    }

    public void updateState(SwerveModuleState desiredState) {
        dt = Timer.getFPGATimestamp() - prevTime;
        prevTime = Timer.getFPGATimestamp();

        state = desiredState;    
        speed = desiredState.speedMetersPerSecond;
        pos += speed * dt;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(pos, state.angle);
    }

    public SwerveModuleState getState() {
        return state;
    }
}
