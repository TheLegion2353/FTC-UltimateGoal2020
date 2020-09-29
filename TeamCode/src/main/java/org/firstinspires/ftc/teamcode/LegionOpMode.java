package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Slide DriveTrain", group="Driver Controlled")
public class LegionOpMode extends OpMode {
    public final double P = 0;
    public final double I = 0;
    public final double D = 0;
    public final double F = 0;

    private ElapsedTime clock = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private double elapsedTime = 0;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor strafeDrive = null;
    private DcMotorEx wobbleArm = null;
    private Servo wobbleGrab = null;
    @Override
    public void init() {
        leftDrive = hardwareMap.get(DcMotor.class, "lMotor");
        rightDrive = hardwareMap.get(DcMotor.class, "rMotor");
        strafeDrive = hardwareMap.get(DcMotor.class, "sMotor");
        wobbleArm = (DcMotorEx)hardwareMap.get(DcMotor.class, "waMotor");
        wobbleGrab = hardwareMap.get(Servo.class, "wgServo");

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArm.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(P, I, D, F));
    }

    @Override
    public void loop() {
        elapsedTime = clock.time();
        clock.reset();
        
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
        strafePower -= gamepad1.left_stick_x;

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        strafeDrive.setPower(strafePower);
    }

}
