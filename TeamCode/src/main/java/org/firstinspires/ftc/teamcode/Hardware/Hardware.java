package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.Motor;
import org.firstinspires.ftc.teamcode.Sensors.BNO055_IMU;
import org.firstinspires.ftc.teamcode.Sensors.UltrasonicSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.GoldDetector;

public class Hardware implements Constants {

    HardwareMap hwMap;
    public AutonomousOpMode auto;
    public Telemetry telemetry;

    public BNO055_IMU imu;
    public UltrasonicSensor rangeSensor;

    public Drivetrain dt;
    public GoldDetector goldFind;
    public DigitalChannel limit;

    public Motor
            frontLeft = new Motor(dtKP,dtKI,dtKD),
            frontRight = new Motor(dtKP,dtKI,dtKD),
            backLeft = new Motor(dtKP,dtKI,dtKD),
            backRight = new Motor(dtKP,dtKI,dtKD),
            pivot1 = new Motor(0,0,0),
            pivot2 = new Motor(0,0,0);

    public Motor[] dtmotors;
    public Motor[] pivotMotors = {pivot1, pivot2};

    public void init (HardwareMap hardwareMap){
        hwMap = hardwareMap;
        imu = new BNO055_IMU("imu",this);

        frontLeft.init(hwMap,"FrontLeft");
        frontRight.init(hwMap,"FrontRight");
        backLeft.init(hwMap,"BackLeft");
        backRight.init(hwMap,"BackRight");

        dt = new Drivetrain(this);
        goldFind = new GoldDetector(this);
    }

    public void setAuto(AutonomousOpMode auto){
        this.auto = auto;
    }

    public void setTelemetry (Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public HardwareMap getHardwareMap()
    {
        return hwMap;
    }
    




}
