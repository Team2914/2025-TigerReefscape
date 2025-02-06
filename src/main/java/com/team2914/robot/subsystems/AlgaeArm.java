package com.team2914.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.team2914.robot.Constants.ArmConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase; 

public class AlgaeArm extends SubsystemBase {

    private AlgaeArm instance;
    private SparkMax armMotor;


    public AlgaeArm getInstance() {
        if (instance == null) {
            this.instance = new AlgaeArm();
        }

        return instance;
    }

    private AlgaeArm() {
        armMotor = new SparkMax(ArmConstants.armMotor, MotorType.kBrushless);
       
        armMotor.setCANTimeout(250);
        SparkMaxConfig armConfig = new SparkMaxConfig();
        armConfig.voltageCompensation(10);
        armConfig.smartCurrentLimit(ArmConstants.armMotor);
        armConfig.idleMode(IdleMode.kBrake);
        armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
        @Override

        public void periodic() {
        }
        
        public void runArm(double speed){
            armMotor.set(speed);
        }
}