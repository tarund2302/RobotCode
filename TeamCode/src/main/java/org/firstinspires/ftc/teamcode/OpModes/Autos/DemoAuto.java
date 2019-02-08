package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Utils.Direction;

public class DemoAuto extends LinearOpMode implements Constants,AutonomousOpMode {

    private Hardware robot = new Hardware();

    @Override
    public boolean getOpModeIsActive() {
        return opModeIsActive();
    }

    @Override
    public Telemetry getTelemetry() {
        return telemetry;
    }

    @Override
    public void runOpMode() {
        robot.setAuto(this);
        robot.setTelemetry(telemetry);
        robot.init(hardwareMap);
        robot.goldFind.setAlignSettings(ALIGN_POSITION,1000);

        waitForStart();
        robot.dt.reset();
        robot.goldFind.startOpenCV(hardwareMap);

        while(getOpModeIsActive() && !robot.goldFind.isFound()){
            robot.dt.rotate(0.33,Direction.RIGHT);
            telemetry.addData("Aligned:", robot.goldFind.getAligned());
            telemetry.addData("Pos:",robot.goldFind.getXPosition());
            telemetry.update();
        }
        sleep(1000);
        robot.dt.stop();
        robot.goldFind.demoAlignGold();
    }


}
