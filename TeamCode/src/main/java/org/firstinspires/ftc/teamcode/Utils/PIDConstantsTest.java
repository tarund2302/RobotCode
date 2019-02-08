package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

import ftc.library.MaelstromControl.PIDController;

@TeleOp(name = "PIDConstantsTest")
public class PIDConstantsTest extends OpMode implements Constants {

    private Hardware robot = new Hardware();

    public void init() {
        robot.init(hardwareMap);
        telemetry.addLine("PID Constant");
        telemetry.addData("Angle:",robot.imu.getRelativeYaw());
        telemetry.update();
    }

    public void loop() {
        double kp = 0;
        double ki = 0;
        double kd = 0;
        double distance = 0;
        double pidIncrement = .0005;
        Direction direction = Direction.UNKNOWN;
        //PIDController pidTester = new PIDController(kp,ki,kd);

        if (gamepad1.dpad_up) kp += pidIncrement;
        else if (gamepad1.dpad_down) kp -= pidIncrement;

        if (gamepad1.dpad_right) ki += pidIncrement;
        else if (gamepad1.dpad_left) ki -= pidIncrement;

        if (gamepad1.right_stick_button) kd += pidIncrement;
        else if (gamepad1.left_stick_button) kd -= pidIncrement;

        if (gamepad1.left_stick_button && gamepad1.right_stick_button){
            robot.dt.distanceDrive.setPID(kp,ki,kd);
        }

        if(gamepad1.right_bumper) direction = Direction.FORWARD;
        else if(gamepad1.left_bumper) direction = Direction.BACKWARD;

        if(gamepad1.right_trigger > 0) distance += 0.5;
        else if(gamepad1.left_trigger > 0)distance -= 0.5;

        if(gamepad1.start) distance = 0;

        if(gamepad1.a){
            robot.dt.distanceDrive.setTarget(distance);
            double target  = robot.dt.distanceDrive.getTarget();
            robot.dt.drive(target,direction);
        }
        else if(gamepad1.x) robot.dt.stop();

        telemetry.addData("Kp:",kp);
        telemetry.addData("Ki:",ki);
        telemetry.addData("Kd:",kd);
        telemetry.addData("Target:",distance);
        telemetry.addData("Direction:",direction);
    }
}
