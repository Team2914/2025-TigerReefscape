package com.team2914.robot.subsystems;

import com.team2914.lib.TigerController;
import com.team2914.robot.Constants.IOConstants;
import com.team2914.robot.subsystems.drivetrain.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class DriverController extends TigerController {
    public static DriverController instance = null;
    
    private static Drivetrain drivetrain = null;

    private boolean fieldRelative = false;
    

    private DriverController(int port) {
        super(port);

        drivetrain = Drivetrain.getInstance();
    }

    public static DriverController getInstance() {
        if (instance == null) {
            instance = new DriverController(IOConstants.kDriverControllerPort);
        }

        return instance;
    }

    @Override
    public void configureButtonBindings() {

        commandController.b()
            .onTrue(new RunCommand(() -> {
                fieldRelative = !fieldRelative;

                SmartDashboard.putBoolean("Driver/Field Relative", fieldRelative);
            }, drivetrain));
        
        commandController.start()
            .onTrue(new RunCommand(()->{
                System.exit(0);
            }, drivetrain));

        //TODO: Rotate buttons

        
    }

    @Override
    public double getLeftX() {
        return -MathUtil.applyDeadband(super.getLeftX(), IOConstants.kDriveDeadband);
    }

    @Override
    public double getLeftY() {
        return -MathUtil.applyDeadband(super.getLeftY(), IOConstants.kDriveDeadband);
    }

    @Override
    public double getRightX() {
        return -MathUtil.applyDeadband(super.getRightX(), IOConstants.kDriveDeadband);
    }

}
