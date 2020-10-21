package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.PID;

public class Arm {
    public final double armP = 0.009;
    public final double armI = 0.001;
    public final double armD = 0.001;

    private DcMotor motor = null;
    private PID PIDController = null;

    public Arm(DcMotor m) {
        motor = m;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PIDController = new PID(armP, armI, armD, motor.getCurrentPosition());
    }

    public void update(double position, double elapsedTime) {
        PIDController.setSetPoint(position);
        double power = PIDController.PIDLoop((double)motor.getCurrentPosition(), elapsedTime);
        motor.setPower(power);
    }

    public double getArmPosition() {
        return motor.getCurrentPosition();
    }
}
