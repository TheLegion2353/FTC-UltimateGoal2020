package org.firstinspires.ftc.teamcode;

public class PID {
    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double errorOverTimeMax = 10;
    private double setPoint = 0;
    private double error = 0;
    private double previousError = 0;
    private double errorOverTime = 0;
    
    public PID(double P, double I, double D, double sp) {
        kP = P;
        kI = I;
        kD = D;
        setPoint = sp;
        error = setPoint - error;
        previousError = error;
    }

    public double PIDLoop(double currentPos, double elapsedTime) {
        double processVar = 0;
        calcErrors(currentPos, elapsedTime);
        processVar += calcP(currentPos) + calcI(currentPos) + calcD(currentPos, elapsedTime);
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
    
    private double calcP(double currentPos) {
        return error * kP;
    }
    
    private double calcI(double currentPos) {
        return errorOverTime * kI;
    }
    
    private double calcD(double currentPos, double elapsedTime) {
        return ((error - previousError)/elapsedTime) * kD;
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
