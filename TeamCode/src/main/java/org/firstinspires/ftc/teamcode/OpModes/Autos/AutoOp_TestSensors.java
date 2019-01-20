package org.firstinspires.ftc.teamcode.OpModes.Autos;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@Autonomous(name = "AutoOp")
public class AutoOp_TestSensors extends LinearOpMode implements Constants, AutonomousOpMode
{

    private Hardware robot = new Hardware();

    public boolean getOpModeIsActive()
    {
        return opModeIsActive();
    }

    public Telemetry getTelemetry()
    {
        return telemetry;
    }

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        robot.setAuto(this);
        robot.setTelemetry(telemetry);
        robot.init(hardwareMap);

        /*GoldFinder gold = new GoldFinder(this, robot);*/
        //robot.gold.setAlignSettings(ALIGN_POSITION,1000);
        double goldPos = 0;

        waitForStart();
        runtime.reset();
        robot.dt.reset();

        //test

        //testing for gyro
        /*robot.dt.turnAngle(40,Direction.LEFT);
        robot.dt.stopTime(2500);
        robot.dt.turnAngle(35,Direction.RIGHT);
        robot.dt.stopTime(5000);
        robot.dt.turnAngle(20,Direction.LEFT);
        robot.dt.stopTime(5000);
        robot.dt.turnAngle(30,Direction.LEFT);
        robot.dt.stop();
*/
        /* //testing angle correction
        robot.dt.testAngleCorrection();
        */

/*
        //testing one side turns
        robot.dt.sideTurnAngle(30, "left");
        robot.dt.stopTime(2500);
        robot.dt.sideTurnAngle(-25, "right");
        robot.dt.stopTime(3000);
        robot.dt.sideTurnAngle(90, "left");
        robot.dt.stopTime(2000);;
        robot.dt.sideTurnAngle(30, "right");
        robot.dt.stopTime(2500);
        robot.dt.sideTurnAngle(100, "right");
        robot.dt.stop();
 */
   /*     //testing for encoders
        robot.dt.driveDistance(24);
        robot.dt.stopTime(5000);
        robot.dt.driveDistance(-12);
        robot.dt.stopTime(300);
        robot.dt.driveDistance(10);
        robot.dt.stopTime(3000);
        robot.dt.driveDistance(-10);
*/
/*

        //testing directional drive
        robot.dt.driveTest(24,Direction.FORWARD);
        robot.dt.stopTime(5000);
        robot.dt.driveTest(12,Direction.BACKWARD);
        robot.dt.stopTime(300);
        robot.dt.driveTest(10,Direction.FORWARD);
        robot.dt.stopTime(3000);
        robot.dt.driveTest(10,Direction.BACKWARD);
*/

/*
        //testing for range sensor
        robot.dt.driveTillRangeDistance(4);
        robot.dt.stopTime(5000);
        robot.dt.driveTillRangeDistance(2);
        robot.dt.stopTime(5000);
        robot.dt.driveTillRangeDistance(6);
*/
/*

        robot.gold.startOpenCV(hardwareMap);

        */
/*      while(getOpModeIsActive() && !gold.isFound()){
            robot.dt.rotate(-0.33);
            telemetry.addData("Aligned:", gold.getAligned());
            telemetry.addData("Pos:",gold.getXPosition());
            telemetry.update();
        }*//*

        robot.gold.goldSearch(-0.33);
        sleep(1000);
        robot.drivetrain.stop();
        robot.gold.alignGold();
*/
        //testing elapsed time
 /*       robot.dt.driveForTime(0.25,15000);
        while(getOpModeIsActive() && !(runtime.seconds() > 30)){
            if(runtime.seconds() >= 10){
                robot.dt.rotate(.75);
            }
        }
*/
    }


} //main
