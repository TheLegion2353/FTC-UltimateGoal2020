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
    private double averageVelocity = 0;
    private double averageVoltage = 0;

    public Shooter(Gamepad gp, Telemetry t, DcMotor ... motors) {
        super(gp);
        telemetry = t;
        pid = new PID(0.005, 0.00, 0.0, 0);
        shooter = new HardwareController(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motors);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void driverUpdate() {
        double velocity = (double)(shooter.getPos() - lastPosition) / ((double)(System.currentTimeMillis() - lastTime) / 1000.0d);
        averageVelocity += velocity;
        averageVelocity /= 2;
        double power = pid.PIDLoop(averageVelocity);
        averageVoltage += power;
        averageVoltage /= 2;
        telemetry.addData("Average Power: ", averageVoltage);
        telemetry.addData("Average Velocity", averageVelocity);
        if (power < 0){
            power = 0;
        }
        if (gamepad.y) {
            pid.setSetPoint(1800);
            shooter.setSpeed(averageVoltage);
        } else if (gamepad.b) {
            shooter.setSpeed(0.65);
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
