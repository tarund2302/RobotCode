package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.Motor;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

import ftc.library.MaelstromControl.PIDController;
import ftc.library.MaelstromMotions.MaelstromMotors.MaelstromMotorSystem;
import ftc.library.MaelstromMotions.MaelstromMotors.MotorModel;
import ftc.library.MaelstromWrappers.MaelstromController;

public class Pivot implements Constants {

    private double kp = 0.01;
    private double ki = 0;
    private double kd = 0;

    private double targetPosition;
    private double basePower = 1;
    private double baseDownPower = 1;
    private double rotatorPower = basePower;
    private double downPower = -0.1;

    public PIDController pivotControl = new PIDController(kp,ki,kd,1);
    public PIDController pivotPID = new PIDController(kp,ki,kd,1);

    private double liftPosition = 0;

    Hardware hardware;
    //Robot robot;
    public MaelstromMotorSystem pivot;
    public Pivot(HardwareMap hwMap){

        //pivot = new MaelstromMotorSystem("pivot1","pivot2", MotorModel.NEVEREST60, hwMap);
        pivot = new MaelstromMotorSystem("pivot1","pivot2", "Pivot", DcMotorSimple.Direction.FORWARD, hwMap, MotorModel.NEVEREST60);
        pivot.stopAndReset();
        pivot.runWithoutEncoders();
    }

    public void teleOp(MaelstromController controller){
        kp = (pivotKP * -liftPosition) + 0.001;
        ki = 0.0;
        kd = 0.0;
        rotatorPower = (0.0001 * -liftPosition) + basePower;
        downPower = (0.001 * Math.abs(getPosition()) + baseDownPower);

        if(controller.leftTrigger() > 0) pivot.setPower(1);
        else if(controller.rightTrigger() > 0) pivot.setPower(-1);

        if(controller.leftTrigger() > 0 || controller.rightTrigger() > 0) targetPosition = pivot.getCounts();
        else{
            double currPosition = pivot.getCounts();
            double pidPower = pivotPID.power(targetPosition,currPosition);
            pivot.setPower(pidPower);
        }
        pivotPID.setPID(kp,ki,kd);
    }

    private AutonomousOpMode auto;
    private Telemetry telemetry;

    public Pivot(Hardware hardware){
        this.hardware=hardware;
        auto = hardware.auto;
        telemetry = hardware.telemetry;
    }

    public void driverControl(Gamepad controller) {
        kp = (pivotKP * -liftPosition) + 0.001;
        ki = 0.0;
        kd = 0.0;
        rotatorPower = (0.0001 * -liftPosition) + basePower;
        downPower = (0.001 * Math.abs(getPosition()) + baseDownPower);
        if (controller.left_bumper){
            setPower(-rotatorPower);
        }
        else if (controller.right_bumper) {
            setPower(downPower);
        } else {
            setPower(0);
        }

        if (controller.right_bumper|| controller.left_bumper) targetPosition = getPosition();
        else {
            double currentPosition = getPosition();
            //setPower(pivotControl.power(targetPosition,currentPosition));
        }

        /*if(!hardware.limit.getState()){
            eReset();
        }
*/
        pivotControl.setKP(kp);
        pivotControl.setKI(ki);
        pivotControl.setKD(kd);
    }

    public void driverControl(Gamepad controller,boolean idc) {

        kp = (pivotKP * -liftPosition) + 0.001;
        ki = 0.0;
        kd = 0.0;
        rotatorPower = (0.0005 * -liftPosition) + basePower;
        downPower = (0.005 * Math.abs(getPosition()) + baseDownPower);
        if (controller.dpad_right){
            setPower(-rotatorPower);
        }
        else if (controller.dpad_left) {
            setPower(downPower);
        } else {
            setPower(0);
        }

        if (controller.dpad_left|| controller.dpad_right) targetPosition = getPosition();
        else {
            double currentPosition = getPosition();
            //setPower(pivotControl.power(targetPosition,currentPosition));
        }

        /*if(!hardware.limit.getState()){
            eReset();
        }
*/

        pivotControl.setKP(kp);
        pivotControl.setKI(ki);
        pivotControl.setKD(kd);
    }

    public void scoringPosition(){
        double SCORE_KP = 0.5;
        PIDController scoringPosition = new PIDController(1,0,0,0);
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;

        double ticks = 0;
        double desiredAngle = 120;

        while((opModeIsActive() && (stopState <= 250))){
            double position = getPosition();
            //double power = scoringPosition.power(ticks,position);
            double power = scoringPosition.power(desiredAngle,getAngle());
            double PIDMultiplier = power + (SCORE_KP*Math.sin(getAngle()));

            telemetry.addData("stopstate: ", stopState);
            telemetry.addData("Angle: ", getAngle());
            telemetry.addData("KP*error: ",scoringPosition.returnVal()[0]);
            telemetry.addData("KI*i: ",scoringPosition.returnVal()[1]);
            telemetry.addData("KD*d: ",scoringPosition.returnVal()[2]);
            telemetry.addData("Error: ",scoringPosition.getError());
            telemetry.update();

            hardware.pivot1.setPower(PIDMultiplier);
            hardware.pivot2.setPower(PIDMultiplier);

            if (!hardware.limit.getState() && getAngle() <= 0.5) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }
        }
        stop();
        eReset();
    }

    public void downPosition(){
        PIDController scoringPosition = new PIDController(1,0,0,0);
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;

        double ticks = 0;

        while((opModeIsActive() && (stopState <= 250))){
            double position = getPosition();
            double power = scoringPosition.power(ticks,position);

            telemetry.addData("stopstate: ", stopState);
            telemetry.addData("KP*error: ",scoringPosition.returnVal()[0]);
            telemetry.addData("KI*i: ",scoringPosition.returnVal()[1]);
            telemetry.addData("KD*d: ",scoringPosition.returnVal()[2]);
            telemetry.update();

            hardware.pivot1.setPower(power);
            hardware.pivot2.setPower(power);

            if (Math.abs(ticks-position)<= ENCODER_TOLERANCE) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }
        }
        stop();

    }

    private void eReset(){
        for(Motor motor: hardware.pivotMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    public void stop(){
        for(Motor motor: hardware.pivotMotors) {
            motor.setPower(0);
        }
    }

    public void setLiftPosition (double liftPosition) {
        this.liftPosition = liftPosition;
    }

    public double getPosition() {
        return (hardware.pivotMotors[1].getCounts())+(hardware.pivotMotors[0].getCounts())-1;
    }

    public void setPower(double power){
        for(Motor motor: hardware.pivotMotors) {
            motor.setPower(power);
        }
    }

    public double getRawPower () {
        return hardware.pivotMotors[0].getPower();
    }

    private double getAngle(){
        double angle = 0;
        for(Motor motor: hardware.pivotMotors) {
            angle += motor.getAngle(PIVOT_TICKS_PER_INCH,PIVOT_TICKS_PER_ROTATION);
        }
        return angle;
    }

    public boolean opModeIsActive() {
        return auto.getOpModeIsActive();
    }

}
