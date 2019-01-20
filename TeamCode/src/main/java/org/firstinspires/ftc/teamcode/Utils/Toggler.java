package org.firstinspires.ftc.teamcode.Utils;


import com.qualcomm.robotcore.hardware.Gamepad;

public class Toggler {

    boolean currState = false;
    boolean prevState = false;
    boolean taskState = true;
    Gamepad gamepad;

    public Toggler(Gamepad gamepad){
        this.gamepad = gamepad;
    }


    public boolean toggle(boolean boolState){

        if(boolState){
            currState = true;
        }
        else{
            currState = false;
            if(prevState){
                taskState = !taskState;
            }
        }
        prevState = currState;

        return taskState;
    }

    public boolean aToggle(){return toggle( gamepad.a);}
    public boolean bToggle(){return toggle( gamepad.b);}
    public boolean xToggle(){return toggle( gamepad.x);}
    public boolean yToggle(){return toggle( gamepad.y);}
    public boolean leftBumperToggle(){return toggle(gamepad.left_bumper);}
    public boolean rightBumperToggle(){return toggle(gamepad.right_bumper);}
    public boolean upDpadToggle(){return toggle(gamepad.dpad_up);}
    public boolean downpDpadToggle(){return toggle(gamepad.dpad_down);}
    public boolean leftDpadToggle(){return toggle(gamepad.dpad_left);}
    public boolean rightDpadToggle(){return toggle(gamepad.dpad_right);}
    public boolean leftJoystickToggle(){return toggle(gamepad.left_stick_button);}
    public boolean rightJoystickToggle(){return toggle(gamepad.right_stick_button);}
}
