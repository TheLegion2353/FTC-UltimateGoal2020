package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Slide DriveTrain", group="Driver Controlled")
public class LegionOpMode extends OpMode {
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor strafeDrive = null;
    private DcMotor wobbleArm = null;
    private Servo wobbleGrab = null;
    @Override
    public void init() {
        leftDrive = hardwareMap.get(DcMotor.class, "lMotor");
        rightDrive = hardwareMap.get(DcMotor.class, "rMotor");
        strafeDrive = hardwareMap.get(DcMotor.class, "sMotor");
        wobbleArm = hardwareMap.get(DcMotor.class, "waMotor");
        wobbleGrab = hardwareMap.get(Servo.class, "wgServo");

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        double leftPower = 0;
        double rightPower = 0;
        double strafePower = 0;
        double wobbleMotorPower = 0;
        double wobbleMotorPosition = 0;
        double wobbleGrabPosition = 0;

        leftPower = gamepad1.left_stick_y;
        rightPower = -gamepad1.left_stick_y;

        leftPower -= gamepad1.right_stick_x;
        rightPower -= gamepad1.right_stick_x;
        strafePower = -gamepad1.left_stick_x;

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        strafeDrive.setPower(strafePower);
    }

}
