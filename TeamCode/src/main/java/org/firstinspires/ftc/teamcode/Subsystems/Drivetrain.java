package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.Motor;
import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Sensors.BNO055_IMU;
import org.firstinspires.ftc.teamcode.Utils.Direction;

public class Drivetrain implements Constants {

    public Motor frontLeft;
    public Motor frontRight;
    public Motor backLeft;
    public Motor backRight;

    public Telemetry telemetry;
    private AutonomousOpMode auto;
    private Hardware hardware;
    private BNO055_IMU imu;
    private Gamepad controller = new Gamepad();
    private double yDirection;
    private double xDirection;
    private double angle;
    private double fieldCentric;
    private double desiredAngle;
    private double speeds[];

    PIDController distanceDrive = new PIDController(distanceKP,distanceKI,distanceKD,distanceMaxI);
    PIDController turnAngle =new PIDController(turnBigKP,turnBigKI,turnBigKD,turnBigMaxI);
    PIDController smallTurnAngle = new PIDController(turnKP, turnKI, turnKD,turnMaxI);
    PIDController testTurn = new PIDController(testTurnKP,testTurnKI,testTurnKD, testTurnMaxI);
    PIDController bigTestTurn = new PIDController(bigTestTurnKP,bigTestTurnKI,bigTestTurnKD, bigTestTurnMaxI);
    PIDController rangeDistance = new PIDController(rangeKP, rangeKI, rangeKD, rangeMaxI);
    PIDController turnSide = new PIDController(sideKP, sideKI, sideKD, sideMaxI);
    PIDController bigTurnSide = new PIDController(bigSideKP, bigSideKI, bigSideKD, sideMaxI);
    PIDController angularCorrection = new PIDController(angleCorrectionKP,angleCorrectionKI,angleCorrectionKD, angleCorrectionMaxI);

    public double frontLeftData;
    public double frontRightData;
    public double backLeftData;
    public double backRightData;

    public Drivetrain(Hardware hardware){
        this.hardware = hardware;
        frontLeft = hardware.frontLeft;
        frontRight = hardware.frontRight;
        backLeft = hardware.backLeft;
        backRight = hardware.backRight;
        auto = hardware.auto;
        telemetry = hardware.telemetry;
        imu = hardware.imu;

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    public void leftDrive(double power){
        frontLeft.setPower(power);
        backLeft.setPower(power);
    }
    public void rightDrive(double power){
        frontRight.setPower(power);
        backRight.setPower(power);
    }

    public void drive(double left, double right){
        leftDrive(left);
        rightDrive(right);
    }

    public void drive(double power){
        leftDrive(power);
        rightDrive(power);
    }

    public void drive(Gamepad controller){
        this.controller = controller;
        yDirection = SPEED_MULTIPLIER * controller.left_stick_y;
        xDirection = SPEED_MULTIPLIER * controller.right_stick_x;

        double left = yDirection - xDirection;
        double right = yDirection + xDirection;

        drive(left,right);
    }

    public void mecanum(Gamepad controller){

        double leftY = controller.left_stick_y;
        double leftX = controller.right_stick_x;
        double rightX = controller.right_stick_x;

        double x = -leftY;
        double y = leftX;

        double angle = Math.atan2(y,x);
        double fieldCentric = angle + Math.toRadians(imu.getYaw());
        double adjustedAngle = fieldCentric + Math.PI / 4;

        this.angle = angle;
        this.fieldCentric = fieldCentric;

        double speedMagnitude = Math.hypot(x,y);

        if(Math.abs(rightX) > 0.00001) desiredAngle = imu.getRelativeYaw();

        double speeds[] = {Math.sin(adjustedAngle), Math.cos(adjustedAngle), Math.cos(adjustedAngle), Math.sin(adjustedAngle)};

        speeds[0] = (speeds[0] * speedMagnitude * SPEED_MULTIPLIER) - rightX * SPEED_MULTIPLIER;
        speeds[1] = (speeds[1] * speedMagnitude * SPEED_MULTIPLIER) - rightX * SPEED_MULTIPLIER;
        speeds[2] = (speeds[2] * speedMagnitude * SPEED_MULTIPLIER) + rightX * SPEED_MULTIPLIER;
        speeds[3] = (speeds[3] * speedMagnitude * SPEED_MULTIPLIER) + rightX * SPEED_MULTIPLIER;
        this.speeds = speeds;

        frontLeft.setPower(speeds[0]);
        backLeft.setPower(speeds[1]);
        frontRight.setPower(speeds[2]);
        backRight.setPower(speeds[3]);
    }

    public void stop(){
        leftDrive(0);
        rightDrive(0);
    }

    public void stopTime(long time){
        long startTime = System.nanoTime();
        long stopState = 0;

        while(stopState <= time){
            stop();
            stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
        }
    }

    public void reset(){
        stop();
        for(Motor motor : hardware.dtmotors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        telemetry.addData("Status:",frontLeft.getCounts());
        telemetry.update();
    }

    public void powerDrive(double power, Direction direction){
        power *= direction.value;
        leftDrive(power);
        rightDrive(power);

        while(opModeActive()){
            addTelemetry("Speed:",frontLeft.getPower());
        }
    }

    public void driveForTime(double power, double time, Direction direction){
        long startTime = System.nanoTime();
        long stopState = 0;

        while(stopState <- time && opModeActive()){
            powerDrive(power,direction);
            addTelemetry("Stop State:",stopState);
        }
        stop();
    }

    public void drive(double distance, Direction direction){
        reset();
        distance *= direction.value;
        double counts = distanceToCounts(distance);
        long startTime = System.nanoTime();
        long stopState = 0;
        double initialHeading = imu.getRelativeYaw();

        while(opModeActive() && (stopState <= 1000)){
            double position = frontLeft.getCounts();
            double  distancePower = distanceDrive.power(counts,position);
            double angleCorrection = angularCorrection.power(initialHeading,imu.getRelativeYaw());

            double left = distancePower - angleCorrection;
            double right = distancePower + angleCorrection;

            drive(left,right);

            telemetry.addData("Power", distancePower);
            telemetry.addData("Distance", countsToDistance(position));
            telemetry.addData("Error:", distanceDrive.getError());
            telemetry.addData("Stop State:", stopState);
            telemetry.update();

            stopState = Math.abs(counts- position)<= distanceToCounts(DISTANCE_TOLERANCE) ?
             stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC : startTime;

            if(startTime/NANOSECS_PER_MILISEC >= 5000) break;
        }
        stop();
    }

    public void rotate(double speed, Direction direction){
        speed *= direction.value;
        drive(-speed,speed);
        addTelemetry("Speed:",frontLeft.getPower());
    }

    public void rotateForTime(double power, double time, Direction direction){
        long startTime = System.nanoTime();
        long stopState = 0;

        while(stopState <= time){
            rotate(power,direction);
        }
        stop();
    }

    public void turn(double degrees, Direction direction){
        long startTime = System.nanoTime();
        long stopState = 0;
        degrees *= direction.value;
        double target = imu.getRelativeYaw() + degrees;

        while(opModeActive() && (stopState <= 1000)){
            double position = imu.getRelativeYaw();
            double power = Math.abs(target) < 50 ?
                    smallTurnAngle.power(target,position) :
                    turnAngle.power(target,position);

            drive(-power,power);

            if(Math.abs(target) < 50){
                telemetry.addLine("Small Turn");
                telemetry.addData("Angle:",imu.getRelativeYaw());
                telemetry.addLine("");
                telemetry.addData("KP*error: ", smallTurnAngle.returnVal()[0]);
                telemetry.addData("KI*i: ", smallTurnAngle.returnVal()[1]);
                telemetry.addData("KD*d: ", smallTurnAngle.returnVal()[2]);
                telemetry.addData("Error: ", smallTurnAngle.getError());
                telemetry.addData("Power: ", power);
                telemetry.update();
            }
            else{
                telemetry.addLine("Big Turn");
                telemetry.addData("Angle:",imu.getRelativeYaw());
                telemetry.addLine("");
                telemetry.addData("KP*error: ", turnAngle.returnVal()[0]);
                telemetry.addData("KI*i: ", turnAngle.returnVal()[1]);
                telemetry.addData("KD*d: ", turnAngle.returnVal()[2]);
                telemetry.addData("Error: ", turnAngle.getError());
                telemetry.update();
            }
            stopState = Math.abs(position - target) <= IMU_TOLERANCE ?
              stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC :
               startTime;

            if(startTime/NANOSECS_PER_MILISEC >= 5000) break;
        }
        stop();
    }

    public void sideTurn(double degrees, String side, Direction direction){
        long startTime = System.nanoTime();
        long stopState = 0;

        while(opModeActive() && (stopState <= 1000)){
            double position = imu.getRelativeYaw();
            degrees *= direction.value;
            double power = Math.abs(degrees) < 50 ?
             turnSide.power(degrees,position) :
              bigTurnSide.power(degrees,position);

           if(side == "left") drive(power,0);
           else drive(0,power);

            if(Math.abs(degrees) < 50){
                telemetry.addLine("Small Turn");
                telemetry.addData("Angle:",hardware.imu.getRelativeYaw());
                telemetry.addLine("");
                telemetry.addData("KP*error: ", turnSide.returnVal()[0]);
                telemetry.addData("KI*i: ", turnSide.returnVal()[1]);
                telemetry.addData("KD*d: ", turnSide.returnVal()[2]);
                telemetry.addData("Error: ", turnSide.getError());
                telemetry.addData("Power: ", power);
                telemetry.update();
            }
            else{
                telemetry.addLine("Big Turn");
                telemetry.addData("Angle:",hardware.imu.getRelativeYaw());
                telemetry.addLine("");
                telemetry.addData("KP*error: ", bigTurnSide.returnVal()[0]);
                telemetry.addData("KI*i: ", bigTurnSide.returnVal()[1]);
                telemetry.addData("KD*d: ", bigTurnSide.returnVal()[2]);
                telemetry.addData("Error: ", bigTurnSide.getError());
                telemetry.update();
            }

            stopState = Math.abs(position - degrees) <= IMU_TOLERANCE ?
                    stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC :
                    startTime;

            if(startTime/NANOSECS_PER_MILISEC >= 5000) break;
        }
        stop();
    }
    public void testAngleCorrection(){
        double initialHeading = imu.getRelativeYaw();
        long startTime = System.nanoTime();
        long stopState = 0;

        while(opModeActive() && (stopState <= 1500)){
            double angleCorrectionPower = angularCorrection.power(initialHeading,imu.getRelativeYaw());

            telemetry.addData("Power:",angleCorrectionPower);
            telemetry.addData("Heading:",imu.getRelativeYaw());
            telemetry.addData("Error:", angularCorrection.getError());
            telemetry.update();

            leftDrive(-angleCorrectionPower);
            rightDrive(angleCorrectionPower);

            if(Math.abs(imu.getRelativeYaw() - initialHeading) <= IMU_TOLERANCE){
                stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
            }
            else{
                startTime = System.nanoTime();
            }
        }
        //stop();
    }

    public boolean opModeActive(){return auto.getOpModeIsActive();}

    public double distanceToCounts(double distance){
        return (distance/WHEEL_CIRCUM)*DRIVE_GEAR_REDUCTION *NEVEREST_40_COUNTS_PER_REV;
    }
    public double countsToDistance(double counts){
        return (counts*WHEEL_CIRCUM *DRIVEN_GEAR_REDUCTION)/NEVEREST_40_COUNTS_PER_REV;
    }

    public void addTelemetry(String name, Object value){
        telemetry.addData(name,value);
        telemetry.update();
    }

}
