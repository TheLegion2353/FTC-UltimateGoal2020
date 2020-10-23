package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name="Test Drive", group="Driver Controlled")
public class TestOpMode extends OpMode {
    Robot robot = null;

    @Override
    public void init() {
        robot = new Robot(gamepad1);
        robot.setLeftGroup(hardwareMap.get(DcMotor.class, "lMotor"));
        robot.setRightGroup(hardwareMap.get(DcMotor.class, "rMotor"));
        robot.setSlideGroup(hardwareMap.get(DcMotor.class, "sMotor"));
        robot.setArmMotor(hardwareMap.get(DcMotor.class, "waMotor"));
        robot.setShooter(hardwareMap.get(DcMotor.class, "firstFlywheel"), hardwareMap.get(DcMotor.class, "secondFlywheel"));
        robot.setGrabber(hardwareMap.get(Servo.class, "wgServo"));
    }


    @Override
    public void loop() {
        robot.update(0);
        telemetry.addData("Arm Position: ", robot.getArmPosition());
        telemetry.update();
    }
}