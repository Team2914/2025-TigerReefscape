package com.team2914.lib;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public abstract class TigerController {
    protected final XboxController controller;
    protected final CommandXboxController commandController;

    //protected final Joystick joystick;

    public TigerController(int port) {
        commandController = new CommandXboxController(port);
        controller = commandController.getHID();
        //joystick = new Joystick(port);
    }

    public XboxController getController() {
        return controller;
    }

    public CommandXboxController getCommandController(){
        return commandController;
    }

    public abstract void configureButtonBindings();
    
    public double getLeftTriggerAxis(){
        return controller.getLeftTriggerAxis();
    }

    public boolean getLeftBumper(){
        return controller.getLeftBumper();
    }

    public double getRightTriggerAxis(){
        return controller.getRightTriggerAxis();
    }

    public boolean getRightBumper(){
        return controller.getRightBumper();
    }

    public boolean getXButtonPressed(){
        return controller.getXButtonPressed();
    }

    public boolean getAButtonPressed(){
        return controller.getAButtonPressed();
    }

    public boolean getAButton(){
        return controller.getAButton();
    }

    public boolean getBButon(){
        return controller.getBButton();
    }

    public boolean getYButton(){
        return controller.getYButton();
    }

    public double getLeftX() {
        return controller.getLeftX();
    }

    public double getLeftY() {
        return controller.getLeftY();
    }

    public double getRightX() {
        return controller.getRightX();
    } 

}
