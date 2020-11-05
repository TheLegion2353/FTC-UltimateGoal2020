package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class Robot {
    ArrayList<RobotPart> parts = new ArrayList<RobotPart>();
    private Drivetrain slide = null;
    private Grabber grabber = null;
    private Shooter shooter = null;
    private Intake intake = null;
    private ElapsedTime clock = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private Gamepad gamepad = null;

    public Robot() {

    }

    public Robot(Gamepad gp) {
        gamepad = gp;
        slide = new Drivetrain(Drivetrain.ControlType.ARCADE, gamepad);
    }

    public void update() {
        double time = (double)clock.time(TimeUnit.MILLISECONDS) / 1000.0;

        for (RobotPart part : parts) {
            part.update();
        }

        if (slide != null) {
            slide.update();
        }

        if (shooter != null) {
            shooter.update();
        }

        if (grabber != null) {
            grabber.update();
        }

        if (intake != null) {
            intake.update();
        }

        clock.reset();
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
        parts.add(new Arm(gamepad, m));
    }

    public void setShooter(DcMotor ... motors) {
        shooter = new Shooter(gamepad, motors);
    }

    public void setGrabber(Servo s) {
        grabber = new Grabber(gamepad, s);
    }

    public void setIntake(DcMotor ... motors) {
        intake = new Intake(gamepad, motors);
    }
}
