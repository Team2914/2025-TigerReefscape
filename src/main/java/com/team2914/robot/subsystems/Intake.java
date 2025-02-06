package com.team2914.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private static Intake instance;
    public SparkMax spinner;
    
    private Intake() {
        spinner = new SparkMax(15, MotorType.kBrushless);
    }

    public void spin(boolean spinning, boolean invert){
        if (spinning) {
            spinner.set(invert ? 0.3 : -0.3);
        } else {
            spinner.set(0);
        }
        
    }

    public static Intake getInstance() {

        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }
}
