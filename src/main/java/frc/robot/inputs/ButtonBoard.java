package frc.robot.inputs;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonBoard {
    
    private CommandGenericHID controller;

    public ButtonBoard(int port) {
        controller = new CommandGenericHID(port);
    }

    public Trigger joy_L() {return controller.axisLessThan(0, 0);}
    public Trigger joy_R() {return controller.axisGreaterThan(0, 0);}
    public Trigger joy_U() {return controller.axisGreaterThan(1, 0);}
    public Trigger joy_D() {return controller.axisLessThan(1, 0);}

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
}
