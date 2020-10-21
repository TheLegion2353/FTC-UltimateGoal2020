package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name="Test Drive", group="Driver Controlled")
public class TestOpMode extends OpMode {
    Robot robot = new Robot();
    private ElapsedTime clock = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double elapsedTime = 0;

    @Override
    public void init() {
        elapsedTime = 0;
        robot.setLeftGroup(hardwareMap.get(DcMotor.class, "lMotor"));
        robot.setRightGroup(hardwareMap.get(DcMotor.class, "rMotor"));
        robot.setSlideGroup(hardwareMap.get(DcMotor.class, "sMotor"));
        robot.setArmMotor(hardwareMap.get(DcMotor.class, "waMotor"));
    }

    @Override
    public void loop() {
        elapsedTime = (double)clock.time(TimeUnit.MILLISECONDS) / 1000.0;
        clock.reset();
        robot.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y, 0, elapsedTime);
        telemetry.addData("Arm Position: ", robot.getArmPosition());
        telemetry.update();
    }
}