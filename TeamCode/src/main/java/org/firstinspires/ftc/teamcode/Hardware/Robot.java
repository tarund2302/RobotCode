package org.firstinspires.ftc.teamcode.Hardware;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.GoldPosition;
import org.firstinspires.ftc.teamcode.Subsystems.GoldDetector;

import ftc.library.MaelstromControl.PIDPackage;
import ftc.library.MaelstromDrivetrains.DrivetrainModels;
import ftc.library.MaelstromDrivetrains.MaelstromDrivetrain;
import ftc.library.MaelstromMotions.MaelstromMotors.MotorModel;
import ftc.library.MaelstromRobot;
import ftc.library.MaelstromSensors.MaelstromIMU;

public class Robot extends MaelstromRobot implements Constants {
    public GoldDetector goldfind;
    public GoldAlignDetector goldAlignDetector;
    public void initHardware(HardwareMap hwMap) {
        imu = new MaelstromIMU("imu",hwMap);
        dt = new MaelstromDrivetrain(DrivetrainModels.ARCADE,2.0,dtKP,dtKI,dtKD,hwMap, MotorModel.NEVEREST40);
        goldfind = new GoldDetector(this);
    }

    @Override
    public PIDPackage pidPackage() {
        PIDPackage pidPackage = new PIDPackage();
        pidPackage.setDtKp(dtKP);
        pidPackage.setDtKi(dtKI);
        pidPackage.setDtKd(dtKD);
        pidPackage.setDistanceKp(distanceKP);
        pidPackage.setDistanceKi(distanceKI);
        pidPackage.setDistanceKd(distanceKD);
        pidPackage.setTurnKp(turnKP);
        pidPackage.setTurnKi(turnKI);
        pidPackage.setTurnKd(turnKD);
        pidPackage.setSideKp(sideKP);
        pidPackage.setSideKi(sideKI);
        pidPackage.setSideKd(sideKD);
        return pidPackage;
    }

    public void startOpenCV(HardwareMap hwMap){
        goldAlignDetector = new GoldAlignDetector();
        goldAlignDetector.init(hwMap.appContext, CameraViewDisplay.getInstance());
        goldAlignDetector.useDefaults();
        goldAlignDetector.alignSize = 5;
        goldAlignDetector.downscale = 0.4;
        goldAlignDetector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        goldAlignDetector.maxAreaScorer.weight = 0.005;
        goldAlignDetector.ratioScorer.weight = 5;
        goldAlignDetector.ratioScorer.perfectRatio = 1.0;
        goldAlignDetector.enable();
    }

    public void setAlignSettings(int offset, int width){
        goldAlignDetector = new GoldAlignDetector();
        goldAlignDetector.setAlignSettings(offset,width);
    }
    public GoldPosition getGold(double position){
        if(position == 300) return GoldPosition.MIDDLE;
        else if(!goldAlignDetector.isFound()) return GoldPosition.LEFT;
        if(position > 300) return GoldPosition.RIGHT;
        else return GoldPosition.UNKNOWN;
    }
/*    public void startOpenCV (HardwareMap hardwareMap){
        init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        useDefaults();
        //Optional Tuning
        alignSize = 5; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        downscale = 0.4; // How much to downscale the input frames
        areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        maxAreaScorer.weight = 0.005;
        ratioScorer.weight = 5;
        ratioScorer.perfectRatio = 1.0;
        enable();
    }*/



}


