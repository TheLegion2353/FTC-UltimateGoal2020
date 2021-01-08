package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PID;

public class Shooter extends RobotPart{
    private HardwareController shooter = null;
    private PID pid = null;
    private double lastPosition = 0;
    private double lastTime = 0;
    private double lastVelocity = 0;
    private Telemetry telemetry = null;

    public Shooter(Gamepad gp, Telemetry t, DcMotor ... motors) {
        super(gp);
        telemetry = t;
        pid = new PID(0.002, 0.001, 0.00001, 0);
        shooter = new HardwareController(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motors);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void driverUpdate() {
        double velocity = (double)(shooter.getPos() - lastPosition) / ((double)(System.currentTimeMillis() - lastTime) / 1000.0d);
        double power = pid.PIDLoop(velocity);
        telemetry.addData("Power: ", power);
        telemetry.addData("Velocity", velocity);
        if (power < 0){
            power = 0;
        }
        if (gamepad.y) {
            pid.setSetPoint(1776);
            shooter.setSpeed(power);
        } else if (gamepad.b) {
            shooter.setSpeed(1);
        } else {
            pid.setSetPoint(0);
            shooter.setSpeed(0);
        }
        lastPosition = shooter.getPos();
        lastTime = System.currentTimeMillis();
    }

    @Override
    public void autonomousUpdate() {

    }

    public boolean setSpeed(double s) {
        shooter.setSpeed(s);
        double velocity = (double)(shooter.getPos() - lastPosition) / ((double)(System.currentTimeMillis() - lastTime) / 1000.0d);
        if (Math.abs(lastVelocity - velocity) <= 0.05d) {
            return true;
        }
        lastVelocity = (double)(shooter.getPos() - lastPosition) / ((double)(System.currentTimeMillis() - lastTime) / 1000.0d);
        lastPosition = shooter.getPos();
        lastTime = System.currentTimeMillis();
        return false;
    }
}
