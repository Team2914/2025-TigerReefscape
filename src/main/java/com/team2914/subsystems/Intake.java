package com.team2914.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.team2914.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private static Intake instance;
  public SparkMax spinner;

  private Intake() {
    spinner = new SparkMax(IntakeConstants.spinnerCanId, MotorType.kBrushless);
  }

  public void spin(boolean spinning, boolean invert) {
    if (spinning) {
      spinner.set(invert ? IntakeConstants.intakeSpeed : -IntakeConstants.intakeSpeed);
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
