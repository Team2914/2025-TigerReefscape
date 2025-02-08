package com.team2914.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.team2914.Configs.AlgaeArmConfig;
import com.team2914.Constants.ArmConstants;
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

    armMotor.configure(
        AlgaeArmConfig.armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void runArm(double speed) {
    armMotor.set(speed);
  }
}
