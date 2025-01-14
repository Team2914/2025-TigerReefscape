// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2914.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.team2914.robot.subsystems.DriverController;
import com.team2914.robot.subsystems.drivetrain.Drivetrain;

// import edu.wpi.first.math.controller.ClosedLoopController;
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
    //private final Vision vision = Vision.getInstance();
    private final SendableChooser<Command> autoChooser;


    private final DriverController driverController = DriverController.getInstance();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        driverController.configureButtonBindings();

        drivetrain.setDefaultCommand(
            new RunCommand(
                () -> drivetrain.drive(
                    driverController.getLeftY(), 
                    driverController.getLeftX(), 
                    driverController.getRightX(), 
                    false, 
                    false), 
                drivetrain));

        /* 
        vision.setDefaultCommand(new RunCommand(() -> {
            if(driverController.getAButtonPressed() && vision.hasTargets()){

                PhotonTrackedTarget target = vision.getBestTarget();

                if(target == null) return;

                if(!VisionConstants.closeAllignable(target.getFiducialId())) return;

                ClosedLoopController controlller = MiscUtil.ClosedLoopControllerFromConstants(new PIDConstants(9, 0, 0));


                //drivetrain.drive(-controller.calculate(range, ))
                 
                while (vision.hasTargets() && !(target.getYaw() <= 1.5 && target.getYaw() > 0)) { //Fps Issue I think -j
                    double rotationSpeed = controlller.calculate(target.getYaw(), Math.PI);
                    drivetrain.drive(0, 0, rotationSpeed, false, false);
                }
               

                //Position off of target (https://docs.photonvision.org/en/latest/docs/programming/photonlib/using-target-data.html)

                
                
                /* 
                System.out.println(PhotonUtils.getYawToPose(drivetrain.getPose(), PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), null, null)));
                if(target.getYaw() <= 1.5) return;
                double rotationSpeed = controlller.calculate(target.getYaw(), Math.PI);
                drivetrain.drive(0, 0, rotationSpeed, false, false);
                
                
            }
            
        
        }, vision));
        */

        autoChooser = AutoBuilder.buildAutoChooser();
        
        
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
