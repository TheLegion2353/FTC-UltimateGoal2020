package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Slide DriveTrain", group="Driver Controlled")
public class LegionOpMode extends OpMode {
    public final double P = 10;
    public final double I = 0;
    public final double D = 0;
    public final double F = 0;

    private double leftPower = 0;
    private double rightPower = 0;
    private double strafePower = 0;
    private double wobbleMotorPower = 0;
    private double wobbleMotorPosition = 0;

    private boolean isRBDown = false;

    private int wobbleGrabPosition = 0;

    private ElapsedTime clock = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double elapsedTime = 0;

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor strafeDrive = null;
    private DcMotorEx wobbleArm = null;
    private Servo wobbleGrab = null;
    @Override
    public void init() {
        wobbleGrabPosition = 0;
        leftDrive = hardwareMap.get(DcMotor.class, "lMotor");
        rightDrive = hardwareMap.get(DcMotor.class, "rMotor");
        strafeDrive = hardwareMap.get(DcMotor.class, "sMotor");
        wobbleArm = (DcMotorEx)hardwareMap.get(DcMotor.class, "waMotor");
        wobbleGrab = hardwareMap.get(Servo.class, "wgServo");

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafeDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobbleArm.setTargetPosition(0);
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArm.setPower(1);

        wobbleGrab.setPosition(0);

        wobbleArm.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(P, I, D, F));
    }

    @Override
    public void loop() {
        resetClock();
        controls();
        setPowers();
    }

    private void resetClock() {
        elapsedTime = clock.time(TimeUnit.MILLISECONDS) / 1000;
        clock.reset();
    }

    private void controls() {
        leftPower = 0;
        rightPower = 0;
        strafePower = 0;
        wobbleMotorPower = 0;

        slideDriveControl();
        wobbleArmControl();
        wobblePositionControl();
    }

    private void setPowers() {
        leftDrive.setPower(-leftPower);
        rightDrive.setPower(-rightPower);
        strafeDrive.setPower(strafePower);
    }

    private void slideDriveControl() {
        leftPower = gamepad1.left_stick_y;
        rightPower = -gamepad1.left_stick_y;
        leftPower -= gamepad1.right_stick_x;
        rightPower -= gamepad1.right_stick_x;
        strafePower -= gamepad1.left_stick_x;
    }

    private void wobbleArmControl() {
        wobbleMotorPower = gamepad1.right_trigger - gamepad1.left_trigger;
        wobbleMotorPosition += wobbleMotorPower * elapsedTime * 10;
        wobbleArm.setTargetPosition((int)wobbleMotorPosition);
        telemetry.addData("Current Arm Position: ", wobbleArm.getCurrentPosition());
        telemetry.addData("Target Arm Position: ", wobbleMotorPosition);
        telemetry.addData("Wobble Arm Power: ", wobbleMotorPower);
        telemetry.addData("Elapsed Time: ", elapsedTime);
    }

    private void wobblePositionControl() {
        if (gamepad1.right_bumper) {
            if (!isRBDown) {
                //gets run only once when first pressed
                wobbleGrabPosition++;
                if (wobbleGrabPosition > 1) {
                    wobbleGrabPosition = 0;
                }
            }
            isRBDown = true;
        } else {
            isRBDown = false;
        }

        switch (wobbleGrabPosition) {
            case 0:
                wobbleGrab.setPosition(0);
                break;
            case 1:
                wobbleGrab.setPosition(1);
                break;
            default:
                wobbleGrabPosition = 0;
                break;
        }
    }
}
