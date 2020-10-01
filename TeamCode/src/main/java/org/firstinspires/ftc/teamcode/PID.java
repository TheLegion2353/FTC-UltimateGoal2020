package org.firstinspires.ftc.teamcode;

public class PID {
    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double setPoint = 0;

    public PID(double P, double I, double D) {
        kP = P;
        kI = I;
        kD = D;
    }

    public double PIDLoop(double currentPos) {
        double processVar = 0;
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
}
