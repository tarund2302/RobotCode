package org.firstinspires.ftc.teamcode.OpModes.LibraryTest;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

import ftc.library.MaelstromWrappers.MaelstromOp;

@TeleOp(name = "LibTeleop",group = "LibTest")
public class TeleopLib extends MaelstromOp implements Constants {
    private Robot robot = new Robot();
    @Override
    public void init() {
        robot.initHardware(hardwareMap);
        feed.add("Testing new library");
        feed.add("Angle:",robot.imu.getRelativeYaw());
        feed.update();
    }

    @Override
    public void loop() {
        robot.driveTeleop(controller1,0.5);

        feed.add("Front Left:",robot.dt.leftDrive.motor1.getVelocity());
        feed.add("Back Left:",robot.dt.leftDrive.motor2.getPower());
        feed.add("Front Right:",robot.dt.leftDrive.motor3.getPower());
        feed.add("Back Right:",robot.dt.leftDrive.motor4.getPower());
        feed.add("Angle:",robot.imu.getRelativeYaw());
        feed.update();
    }
}
