package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;

public class Robot {
    private Drivetrain slide = null;
    private Arm arm = null;
    private Grabber grabber = null;
    private Shooter shooter = null;
    private ElapsedTime clock = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double elapsedTime = 0;
    private Gamepad gamepad = null;

    public Robot(Gamepad gp) {
        gamepad = gp;
        elapsedTime = 0;
        slide = new Drivetrain(Drivetrain.ControlType.ARCADE, gamepad);
    }

    public void update(double armPosition) {
        elapsedTime = (double)clock.time(TimeUnit.MILLISECONDS) / 1000.0;
        clock.reset();
        slide.update();
        if (arm != null) {
            arm.update(elapsedTime);
        }

        if (shooter != null) {
            shooter.update();
        }

        if (grabber != null) {
            grabber.update();
        }

    }

    public void setLeftGroup(DcMotor ... motors) {
        slide.setLeftGroup(DcMotor.RunMode.RUN_USING_ENCODER, motors);
    }

    public void setRightGroup(DcMotor ... motors) {
        slide.setRightGroup(DcMotor.RunMode.RUN_USING_ENCODER, motors);
    }

    public void setSlideGroup(DcMotor ... motors) {
        slide.setSlideGroup(DcMotor.RunMode.RUN_USING_ENCODER, motors);
    }

    public void setArmMotor(DcMotor m) {
        arm = new Arm(gamepad, m);
    }

    public void setShooter(DcMotor ... motors) {
        shooter = new Shooter(gamepad, motors);
    }

    public void setGrabber(Servo s) {
        grabber = new Grabber(gamepad, s);
    }

    public double getArmPosition() {
        return arm.getArmPosition();
    }
}
