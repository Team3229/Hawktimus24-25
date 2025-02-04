package frc.robot.inputs;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonBoard {
    protected CommandGenericHID controller;

    private static final double kControllerDeadzone = 0.1;

    public ButtonBoard(int port) {
        controller = new CommandGenericHID(port);
    }

    public double joy_X() {return applyDeadzone(controller.getRawAxis(0));}
    public double joy_Y() {return applyDeadzone(controller.getRawAxis(1));}

    public Trigger b_1() {return controller.button(1);}
    public Trigger b_2() {return controller.button(2);}
    public Trigger b_3() {return controller.button(3);}
    public Trigger b_4() {return controller.button(4);}
    public Trigger b_5() {return controller.button(5);}
    public Trigger b_6() {return controller.button(6);}
    public Trigger b_7() {return controller.button(7);}
    public Trigger b_8() {return controller.button(8);}
    public Trigger b_9() {return controller.button(9);}
    public Trigger b_10() {return controller.button(10);}
    public Trigger b_11() {return controller.button(11);}
    public Trigger b_12() {return controller.button(12);}

    //deadzone is probably not used based on the joystick
    private static double applyDeadzone(double input) {
        return (Math.abs(input) > kControllerDeadzone) ? 
        Math.pow(input,3)
        : 0;
    }
}
