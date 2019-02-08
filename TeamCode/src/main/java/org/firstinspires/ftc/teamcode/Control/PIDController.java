package org.firstinspires.ftc.teamcode.Control;

public class PIDController implements Constants{
    private double i=0;
    private double d;
    private double error;
    private double target;
    private double power;
    private double KP;
    private double KI;
    private double KD;
    private double previousError = 0;
    private double maxI;
    private double previousTime = 0;


    public PIDController(double KP, double KI, double KD, double maxI){
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.maxI = maxI;
    }
    public PIDController(double KP, double KI){
        this.KP = KP;
        this.KI = KI;
    }

    public PIDController(double KP, double KI, double KD){
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
    }

    public double power(double target, double currentLoc){
        this.target = target;
        double error = target - currentLoc;
        this.error = error;
        double deltaTime = (System.nanoTime() - previousTime)/NANOSECS_PER_MIN;
        i += Math.abs(currentLoc) > Math.abs(target) * 0.7 ? error*deltaTime : 0;
        d = (error - previousError)/deltaTime;
        previousTime = System.nanoTime();
        previousError = error;
        power = (KP*error) + (KI*i) + (KD*d);
        return power;
    }

    public double[] returnVal(){
        double PID[] = {(KP * error), (KI * i), (KD * d), power};
        return PID;
    }
    public double getI() {
        return i;
    }

    public double getKP() {
        return KP;
    }

    public double getKI() {
        return KI;
    }

    public double getKD() {
        return KD;
    }

    public double getError(){return Math.abs(error);}

    public void setTarget(double target){this.target = target;}

    public double getTarget(){return target;}

    public void setKP(double KP){this.KP = KP;}

    public void setKI(double KI){this.KI = KI;}

    public void setKD(double KD){this.KD = KD;}

    public void setPID(double kp, double ki, double kd){
        setKP(kp);
        setKI(ki);
        setKD(kd);
    }

    public void reset(){
        i = 0;
        previousError = 0;
    }


}

