package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;

public class PID {
    public double kP = 0;
    public double kI = 0;
    public double kD = 0;
    private double errorOverTimeMax = 10;
    private double setPoint = 0;
    private double error = 0;
    private double previousError = 0;
    private double errorOverTime = 0;
    private ElapsedTime clock = null;
    
    public PID(double P, double I, double D, double sp) {
        kP = P;
        kI = I;
        kD = D;
        setPoint = sp;
        error = setPoint - error;
        previousError = error;
        clock = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    public double PIDLoop(double currentPos) {
        double processVar = 0;
        double time = (double)clock.time(TimeUnit.MILLISECONDS) / 1000.0;
        calcErrors(currentPos, time);
        processVar += calcP() + calcI() + calcD(time);
        previousError = error;
        return processVar;
    }

    public void setSetPoint(double sp) {
        setPoint = sp;
    }

    public void updateConst(double P, double I, double D) {
        kP = P;
        kI = I;
        kD = D;
    }
    
    private double calcP() {
        return (error * kP);
    }
    
    private double calcI() {
        return (double)errorOverTime * (double)kI;
    }
    
    private double calcD(double elapsedTime) {
        return ((double)(error - previousError)/(double)elapsedTime) * (double)kD;
    }
    
    private void calcErrors(double currentPos, double elapsedTime) {
        error = setPoint - currentPos;
        errorOverTime += error * elapsedTime;
        if (errorOverTime > errorOverTimeMax) {
            errorOverTime = errorOverTimeMax;
        } else if (errorOverTime < -errorOverTimeMax) {
            errorOverTime = -errorOverTimeMax;
        }
    }
}
