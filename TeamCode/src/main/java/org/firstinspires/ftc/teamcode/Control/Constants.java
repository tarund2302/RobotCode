package org.firstinspires.ftc.teamcode.Control;

public interface Constants {

    //hook positions
    double HOOK_UP_POSITION = 0;
    double HOOK_DOWN_POSITION = 1;

    //marker positions
    double MARKER_UP_POSITION = 1;
    double MARKER_DOWN_POSITION = -1;

    //latch drop positions
    double DROP_UP_POSITION = 1;
    double DROP_DOWN_POSITION = -1;

    //extendo power values
    double EXTENDO_EXTEND_POWER = 0.6;
    double EXTENDO_RETRACT_POWER = -0.6;
    double ACTUATOR_POWER = 0.7;

    double NEVEREST_20_COUNTS_PER_REV = 560; //extendo motor ticks
    double NEVEREST_40_COUNTS_PER_REV = 1120; //drivetrain motor ticks
    double NEVEREST_60_COUNTS_PER_REV = 1680;
    double SPEED_MULTIPLIER = 0.5;

    double WHEEL_DIAMETER_INCHES = 4.0;
    double WHEEL_CIRCUM = WHEEL_DIAMETER_INCHES * Math.PI;
    double DRIVE_GEAR_REDUCTION = 0.5; //gear ratio (driven gear / driving gear)
    double DRIVEN_GEAR_REDUCTION = 2.0; //gear ratio (driving gear / driven gear)
    double COUNTS_PER_INCH = (NEVEREST_40_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUM;

    double DISTANCE_TOLERANCE = .5;
    double IMU_TOLERANCE = 0.5;
    double RANGE_DISTANCE_TOLERANCE = 0.1;
    double HANG_TOLERANCE = 10;

    double ENCODER_TOLERANCE = 50;
    double PIVOT_TICKS_PER_ROTATION = 100;
    double PIVOT_TICKS_PER_INCH = 100;


    double AUTO_RAISE_POSITION = 800;
    double AUTO_LOWER_POSITION = 0;

    //used for encoder drive
    double DRIVE_SPEED = 0.5;
    double TURN_SPEED = 0.2;

    double LED_PERIOD = 10;
    double GAMEPAD_LOCKOUT = 500;

    double NANOSECS_PER_MIN = 6e+10;
    long NANOSECS_PER_MILISEC = 1000000;
    double HALF = 0.5;

    double dtKP = 0.001;
    double dtKI = 0;
    double dtKD = 0;
    double dtMaxI = 1;

    //drive PID
    double distanceKP = 0.004;
    double distanceKI = 0;
    double distanceKD = 0;
    double distanceMaxI = 1;

    //angle correction PID
    double angleCorrectionKP = 0.001;
    double angleCorrectionKI = 0;
    double angleCorrectionKD = 0;
    double angleCorrectionMaxI = 1;

    //turn angle (< 50) PID
    double turnKP = 0.002;
    double turnKI = 0;
    double turnKD = 0;
    double turnMaxI = 1;

    //turn big angle (> 50) PID
    double turnBigKP = 0.013;
    double turnBigKI = 0;
    double turnBigKD = 0;
    double turnBigMaxI = 1;

    //test turn PID
    double testTurnKP = 0.002;
    double testTurnKI = 0;
    double testTurnKD = 0;
    double testTurnMaxI = 1;

    double bigTestTurnKP = 0.013;
    double bigTestTurnKI = 0;
    double bigTestTurnKD = 0;
    double bigTestTurnMaxI = 1;


    //turn one side PID
    double sideKP = 0.001;
    double sideKI = 0;
    double sideKD = 0;
    double sideMaxI = 1;

    double bigSideKP = 0.001;
    double bigSideKI = 0;
    double bigSideKD = 0;
    double bigSideMaxI = 1;

    //opencv goldfind PID
    double alignGoldKP = 0.001;
    double alignGoldKI = 0;
    double alignGoldKD = 0;
    double alignGoldMaxI = 1;

    //range sensor PID
    double rangeKP = 0.001;
    double rangeKI = 0;
    double rangeKD = 0;
    double rangeMaxI = 1;

    double climberKP = 0.001;
    double climberKI = 0;
    double climberKD = 0;
    double climberMaxI = 0;

    double pivotKP = 1e-6;
    double pivotKI = 0;
    double pivotKD = 0;
    double pivotMaxI = 1;

    int ALIGN_POSITION = -100;

}

