package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class PID {
    private double Pval = 0;
    private double Ival = 0;
    private double Dval = 0;
    private double setPoint = 0;

    public PID(DcMotor m, double P, double I, double D) {
        Pval = P;
        Ival = I;
        Dval = D;
    }

    public double PIDLoop(double currentPos) {
        double processVar = 0;
        return processVar;
    }

    public void setSetPoint(double sp) {
        setPoint = sp;
    }
}
