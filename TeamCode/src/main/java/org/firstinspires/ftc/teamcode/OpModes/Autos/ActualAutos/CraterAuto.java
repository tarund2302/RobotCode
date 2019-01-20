package org.firstinspires.ftc.teamcode.OpModes.Autos.ActualAutos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.GoldPosition;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Utils.Direction;

public class CraterAuto extends LinearOpMode implements Constants,AutonomousOpMode {
    private double sampleTurn;
    private double sampleDistance;
    private double wallTurn;
    private double wallDistance;
    private GoldPosition position;
    Hardware robot = new Hardware();

    @Override
    public void runOpMode(){
        robot.setAuto(this);
        robot.setTelemetry(telemetry);
        robot.init(hardwareMap);

        waitForStart();

        if(position == GoldPosition.LEFT){
            sampleTurn = 30;
            sampleDistance = 35;
            wallTurn = 60;
            wallDistance = 50;
        }
        else if(position == GoldPosition.MIDDLE){
            sampleTurn = 0;
            sampleDistance = 40;
            wallTurn = 90;
            wallDistance = 55;
        }
        else if(position == GoldPosition.RIGHT){
            sampleTurn = 30;
            sampleDistance = 35;
            wallTurn = 120;
            wallDistance = 60;
        }
        else if(position == GoldPosition.UNKNOWN){
            sampleTurn = 0;
            sampleDistance = 55;
            wallTurn = 90;
        }
        robot.dt.turn(sampleTurn,Direction.LEFT);
        robot.dt.drive(sampleDistance,Direction.FORWARD);
        robot.dt.drive(5,Direction.BACKWARD);
        robot.dt.turn(wallTurn,Direction.LEFT);
        robot.dt.drive(wallDistance,Direction.FORWARD);
        robot.dt.turn(30,Direction.LEFT);
        robot.dt.drive(65,Direction.FORWARD);
        robot.dt.stopTime(2500);
        robot.dt.drive(70,Direction.BACKWARD);
    }

    @Override
    public boolean getOpModeIsActive() {
        return false;
    }
    @Override
    public Telemetry getTelemetry() {
        return null;
    }
}