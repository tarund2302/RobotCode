package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@Disabled
@TeleOp(name = "Mech Teleop")
public class MechTeleop extends OpMode {
    private Hardware robot = new Hardware();

    //field centric mecanum drive code test
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        robot.dt.mecanum(gamepad1);
    }
}