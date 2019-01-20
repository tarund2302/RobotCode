package org.firstinspires.ftc.teamcode.Control;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Motor implements Constants{
    private DcMotor motor;
    private int previousPos = 0;
    private long previousTime = 0;
    private double rpm = 0;
    private double power;
    private double motorCounts = NEVEREST_40_COUNTS_PER_REV;

    private PIDController PID;

    public Motor(double Kp, double Ki, double Kd){
        this.PID = new PIDController(Kp, Ki, Kd);
    }

    public void init(HardwareMap hardwareMap, String name){
        motor = hardwareMap.dcMotor.get(name);
    }

    public void setPower(double power){
        this.power = power;
        motor.setPower(power);
    }

    public void setMode(DcMotor.RunMode runMode){
        motor.setMode(runMode);
    }

    public double getRPM(){
        int deltaPos = motor.getCurrentPosition() - previousPos;
        double deltaTime = (System.nanoTime() - previousTime)/NANOSECS_PER_MIN;
        if (deltaTime*6e4 > 10) {
            rpm = (deltaPos/ motorCounts)/(deltaTime);
            previousPos = motor.getCurrentPosition();
            previousTime = System.nanoTime();
        }
        return rpm;
    }

    public void setRPM(double rpm){
        power = PID.power(rpm,getRPM());
        motor.setPower((power > 0 && getRPM() < 0) || (power < 0 && getRPM() > 0) ? 0: power);
    }

    public void setSpeed(double speed){
        double rpm = motorCounts * speed;
        power = PID.power(rpm, getRPM());
        motor.setPower((power > 0 && getRPM() < 0) || (power < 0 && getRPM() > 0) ? 0: power);
    }


    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
        motor.setZeroPowerBehavior(behavior);
    }

    public double getAngle (double countsPerInch, double countsPerRev){
        double angle = (360*(motor.getCurrentPosition()) % countsPerInch)/ countsPerRev;
        return angle;
    }

    public void setAngle(double angle, double countsPerInch, double countsPerRev){
        double power = PID.power(angle, getAngle(countsPerInch,countsPerRev));
        motor.setPower(/*(power > 0 && getRPM() < 0) || (power < 0 && getRPM() > 0) ? 0:*/ power);
    }

    public int getCounts(){return motor.getCurrentPosition();}

    public double getPower(){return motor.getPower();}

    public void setDirection(DcMotor.Direction direction){motor.setDirection(direction);}

}

