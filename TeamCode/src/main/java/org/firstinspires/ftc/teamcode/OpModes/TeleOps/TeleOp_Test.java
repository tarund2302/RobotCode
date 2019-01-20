package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class TeleOp_Test extends OpMode implements Constants {
    private Hardware robot = new Hardware();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        robot.dt.drive(gamepad1);

        telemetry.addData("Left Front Speed: ", robot.frontLeft.getPower());
        telemetry.addData("Right Front Speed: ", robot.frontRight.getPower());
        telemetry.addData("Left Back Speed: ", robot.backLeft.getPower());
        telemetry.addData("Right Back Speed: ", robot.backRight.getPower());
        telemetry.update();
    }
}
