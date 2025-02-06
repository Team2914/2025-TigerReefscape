package com.team2914.robot.subsystems.simulations;

import com.studica.frc.AHRS;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimAHRS {
    private AHRS gyro;

    private double prevTime;
    private int dev;
    private SimDouble simAngle;
    private SimBoolean simIsConnected;

    public SimAHRS() {
        gyro = new AHRS(AHRS.NavXComType.kMXP_SPI); 
        
        if (RobotBase.isSimulation()) {
            dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
            simAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
            simIsConnected = new SimBoolean(SimDeviceDataJNI.getSimValueHandle(dev, "Connected"));
        }
            
    }

    public void setSimAngle(double angle) {
        this.simAngle.set(angle);
    }

    public double getAngle() {
        return RobotBase.isReal() ? gyro.getAngle() : simAngle.get();
    }

    public void updateSim(SwerveDriveKinematics kinematics, SwerveModuleState[] states) {
        // thanks to YAGSL
        simAngle.set(simAngle.get() + kinematics.toChassisSpeeds(states).omegaRadiansPerSecond * 180 / Math.PI * (Timer.getFPGATimestamp() - prevTime));
        prevTime = Timer.getFPGATimestamp();
    }

    public AHRS getRealGyro() {
        return gyro;
    }
}