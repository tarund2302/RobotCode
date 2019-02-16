package org.firstinspires.ftc.teamcode.OpModes.Autos.ActualAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.GoldPosition;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Utils.Direction;

@Autonomous(name = "CraterAuto")
public class CraterAuto extends LinearOpMode implements Constants,AutonomousOpMode, Runnable {
    private double sampleTurn;
    private double sampleDistance;
    private double wallTurn;
    private double wallDistance;
/*    private Direction drive;*/
    private Direction turn;
    Hardware robot = new Hardware();
    private double position = robot.goldFind.getXPosition();
    private GoldPosition gold = robot.goldFind.getGold(position);


    @Override
    public void runOpMode(){
        robot.setAuto(this);
        robot.setTelemetry(telemetry);
        robot.init(hardwareMap);

        waitForStart();

        if(gold == GoldPosition.LEFT){
            sampleTurn = 30;
            sampleDistance = 35;
            wallTurn = 60;
            wallDistance = 50;
            turn = Direction.LEFT;
        }
        else if(gold == GoldPosition.MIDDLE){
            sampleTurn = 0;
            sampleDistance = 40;
            wallTurn = 90;
            wallDistance = 55;
            turn = Direction.UNKNOWN;
        }
        else if(gold == GoldPosition.RIGHT){
            sampleTurn = 30;
            sampleDistance = 35;
            wallTurn = 120;
            wallDistance = 60;
            turn = Direction.RIGHT;
        }
        else if(gold == GoldPosition.UNKNOWN){
            sampleTurn = 0;
            sampleDistance = 55;
            wallTurn = 90;
            turn = Direction.UNKNOWN;
        }

        robot.dt.turn(sampleTurn,turn);
        robot.dt.driveForward(sampleDistance);
        robot.dt.drive(5,Direction.BACKWARD);
        robot.dt.turn(wallTurn,Direction.LEFT);

        Thread extender = new Thread();
        extender.start();

        robot.dt.driveForward(wallDistance);
        robot.dt.turn(30,Direction.LEFT);
        robot.dt.driveForward(65);
        robot.dt.stopTime(2500);
        robot.dt.drive(70,Direction.BACKWARD);
    }

    public boolean getOpModeIsActive() {
        return opModeIsActive();
    }
    public Telemetry getTelemetry() {
        return telemetry;
    }

    @Override
    public void run() {

    }
}