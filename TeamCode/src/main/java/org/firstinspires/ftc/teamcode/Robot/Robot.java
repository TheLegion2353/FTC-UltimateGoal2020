package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class Robot {
    ArrayList<RobotPart> parts = new ArrayList<RobotPart>();
    private Drivetrain slide = null;
    private Grabber grabber = null;
    private Shooter shooter = null;
    private Intake intake = null;
    private Elevator elevator = null;
    private ElapsedTime clock = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private Gamepad gamepad = null;
    private Telemetry telemetry = null;
    private ServoWacker whacker = null;
    int xPosition = 0;
    int yPosition = 0;

    public Robot(Gamepad gp, Telemetry t) {
        telemetry = t;
        gamepad = gp;
        slide = new Drivetrain(Drivetrain.ControlType.ARCADE, gamepad);
        parts.add(slide);
    }

    public Robot(Gamepad gp) {
        gamepad = gp;

        slide = new Drivetrain(Drivetrain.ControlType.ARCADE, gamepad);
        parts.add(slide);
    }

    public void move(int x, int y) {
        slide.move(x, y);
    }

    public void update() {
        double time = (double)clock.time(TimeUnit.MILLISECONDS) / 1000.0;

        for (RobotPart part : parts) {
            part.update();
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

        if (elevator != null) {
            elevator.update();
        }

        if (whacker != null) {
            whacker.update();
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
        parts.add(new Arm(gamepad, m, telemetry));
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

    public void addElevator(CRServo crServo) {
        elevator = new Elevator(gamepad, crServo);
    }

    public void addWacker(Servo servo) {
        whacker = new ServoWacker(gamepad, servo);
    }
}
