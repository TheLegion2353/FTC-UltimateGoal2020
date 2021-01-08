package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name="Slide Drive Main", group="Driver Controlled")
public class TestOpMode extends OpMode {
    Robot robot = null;

    @Override
    public void init() {
        robot = new Robot(gamepad1, telemetry);
        robot.setLeftGroup(hardwareMap.get(DcMotor.class, "lMotor"));
        robot.setRightGroup(hardwareMap.get(DcMotor.class, "rMotor"));
        robot.setSlideGroup(hardwareMap.get(DcMotor.class, "sMotor"));
        robot.setArmMotor(hardwareMap.get(DcMotor.class, "waMotor"));
        robot.setShooter(hardwareMap.get(DcMotor.class, "firstFlywheel"));
        robot.setIntake(hardwareMap.get(DcMotor.class, "intakeMotor"));
        robot.setGrabber(hardwareMap.get(Servo.class, "wgServo"));
        robot.addWacker(hardwareMap.get(Servo.class, "servoWacker"));
        robot.addAimMotor(hardwareMap.get(DcMotor.class, "aimMotor"));
        robot.setIMU(hardwareMap.get(BNO055IMU.class, "imu 1"));
    }

    @Override
    public void loop() {
        robot.update();
        telemetry.update();
    }
}