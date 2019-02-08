package org.firstinspires.ftc.teamcode.OpModes.LibraryTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

import ftc.library.MaelstromMotions.MaelstromMotors.Direction;
import ftc.library.MaelstromWrappers.MaelstromLinearOp;

@Autonomous(name = "LibraryTestDrives",group = "Lib")
public class SimpleDrivesLib extends MaelstromLinearOp implements Constants {
    private Robot robot = new Robot();

    public void runLinearOpMode() throws InterruptedException {
        robot.initHardware(hardwareMap);
        while(!opModeIsActive()){
            feed.add("Testing new library");
            feed.add("Angle:",robot.imu.getRelativeYaw());
            feed.update();
        }
        waitForStart();

        robot.driveDistance(20);
        robot.driveDistance(10, 0.8, Direction.FORWARD,2000,2);
        robot.turnAbsolute(90,0.5,Direction.RIGHT,1000);
        robot.turn(50,1.0,Direction.LEFT);

        runSimultaneously(new Runnable() {
            @Override
            public void run() {
                robot.driveDistance(50, 0.5, Direction.BACKWARD, 1000,4);
            }
        }, new Runnable() {
            @Override
            public void run() {
                robot.startOpenCV(hardwareMap);
            }
        });
        robot.goldAlignDetector.disable();

        robot.stop();
        sleep(5);
        robot.sideTurn(90,"right",Direction.RIGHT);

    }
}