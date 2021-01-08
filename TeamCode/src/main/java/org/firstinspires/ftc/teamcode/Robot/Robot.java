package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class Robot {
    ArrayList<RobotPart> parts = new ArrayList<RobotPart>();
    private Arm arm = null;
    private Aimer aim = null;
    private Drivetrain slide = null;
    private Grabber grabber = null;
    private Shooter shooter = null;
    private Intake intake = null;
    private ElapsedTime clock = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private Gamepad gamepad = null;
    private Telemetry telemetry = null;
    private ServoWacker whacker = null;
    private boolean isWaiting = false;
    private double endTime = 0;
    double xPosition = 0; //inches
    double yPosition = 0;
    double angle = 0;

    public Robot(Gamepad gp, Telemetry t) {
        telemetry = t;
        gamepad = gp;
        slide = new Drivetrain(Drivetrain.ControlType.ARCADE, gamepad, telemetry);
        parts.add(slide);
    }

    public Robot(Gamepad gp) {
        gamepad = gp;
        slide = new Drivetrain(Drivetrain.ControlType.ARCADE, gamepad, telemetry);
        parts.add(slide);
    }

    public boolean move(double x, double y) {
        return slide.move(x, y);
    }

    public boolean move(double x, double y, double a) {
        return slide.move(x, y, a);
    }

    public boolean moveExact(double x, double y, double a) {
        return slide.moveExact(x, y, a);
    }

    public void update() {
        double time = (double)clock.time(TimeUnit.MILLISECONDS) / 1000.0;

        for (RobotPart part : parts) {
            part.update();
        }

        if (arm != null) {
            arm.update();
            telemetry.addData("Arm Position: ", arm.getArmPosition());
        }

        if (aim != null) {
            aim.update();
            telemetry.addData("Aim Position: ", aim.getArmPosition());
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
        arm = (new Arm(gamepad, m, telemetry));
    }

    public void setShooter(DcMotor ... motors) {
        shooter = new Shooter(gamepad, telemetry, motors);
    }

    public void setGrabber(Servo s) {
        grabber = new Grabber(gamepad, s);
    }

    public void setIntake(DcMotor ... motors) {
        intake = new Intake(gamepad, motors);
    }

    public void addWacker(Servo servo) {
        whacker = new ServoWacker(gamepad, servo);
    }

    public void addAimMotor(DcMotor m) {
        aim = (new Aimer(gamepad, m, telemetry));
    }

    public void setPosition(double x, double y, double a) {
        xPosition = x;
        yPosition = y;
        angle = a;
        slide.setPosition(x, y, a);
    }

    public boolean setArmPosition(int pos) {
        return arm.setArmPosition(pos);
    }

    public void setGrab(int grab) {
        grabber.setGrab(grab);
    }

    public boolean getIsWaiting() {
        telemetry.addData("Time: ", System.currentTimeMillis());
        return System.currentTimeMillis() < endTime;
    }

    public void wait(double t) {
        slide.zeroMovement();
        endTime = t + System.currentTimeMillis();
    }

    public boolean setAngle(double a) {
        return slide.moveAngle(a);
    }

    public void setIMU(BNO055IMU mu) {
        slide.setIMU(mu);
    }

    public boolean setShooterSpeed(double s) {
        return shooter.setSpeed(s);
    }

    public boolean setAimer(double a) {
        return aim.setAngle(a);
    }

    public void setWhacker(double p) {
        whacker.setPosition(p);
    }

    public void setGrabber(int p) {
        grabber.setGrab(p);
    }
}
