// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2914.robot;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;
import com.team2914.robot.Constants.VisionConstants;
import com.team2914.robot.subsystems.DriverController;
import com.team2914.robot.subsystems.Drivetrain;
import com.team2914.robot.subsystems.Intake;
import com.team2914.robot.subsystems.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final Vision vision = Vision.getInstance();
    private final SendableChooser<Command> autoChooser;


    private final DriverController driverController = DriverController.getInstance();
    private final Intake intake = Intake.getInstance();


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        driverController.configureButtonBindings();

        drivetrain.setDefaultCommand(
            new RunCommand(
                () -> drivetrain.drive(
                    driverController.getLeftY() / 2, 
                    driverController.getLeftX() / 2,
                    driverController.getRightX() / 2, 
                    true), 
                drivetrain));

        intake.setDefaultCommand(new RunCommand(() -> {
                if (driverController.getLeftBumper()) {
                    intake.spin(true, false);
                }

                if (driverController.getRightBumper()) {
                    intake.spin(true, true);
                } else if(!driverController.getLeftBumper()) {
                    intake.spin(false, false);
                }

            }   
            , intake)
        );

        
        vision.setDefaultCommand(new RunCommand(() -> {
            if(driverController.getAButtonPressed() && vision.hasTargets()){

                PhotonTrackedTarget target = vision.getBestTarget();

                if(target == null) return;

                if(!VisionConstants.closeAllignable(target.getFiducialId())) return;
                 
                PIDController controller = new PIDController(1, 0, 0);

                double rotationSpeed = controller.calculate(target.getYaw()/*, Math.PI */);

                drivetrain.drive(0, 0, rotationSpeed, false);

                controller.close();
                
            }
            
        
        }, vision));
        

        autoChooser = AutoBuilder.buildAutoChooser();
        
        
        SmartDashboard.putData("Set Auto", autoChooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        //return new RunCommand(() -> {}, drivetrain);
        return autoChooser.getSelected();
    }
}
