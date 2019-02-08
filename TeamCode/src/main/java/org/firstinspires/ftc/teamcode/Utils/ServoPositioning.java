package org.firstinspires.ftc.teamcode.Utils;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@TeleOp(name = "ServoTesting")
public class ServoPositioning extends OpMode{

    public Servo servo;
    //double currPosition = servo.getPosition();
    double position = 0;
    @Override
    public void init() {
        servo = hardwareMap.servo.get("servo");
        telemetry.addLine("Servo positing test");
        //servo.setPosition(0);
    }

    @Override
    public void loop() {
        double increment = 0.1;
        double specificIncrement = 0.01;

        if(gamepad1.dpad_right) position += increment;
        else if(gamepad1.dpad_left) position -= increment;

        if(gamepad1.dpad_up) position += specificIncrement;
        else if(gamepad1.dpad_down) position -= specificIncrement;

        if(gamepad1.a) servo.setPosition(position);
        else if(gamepad1.x) position = 0;

        telemetry.addData("Servo position:", servo.getPosition());
        telemetry.addData("Set position:", position);
        telemetry.update();
    }
}