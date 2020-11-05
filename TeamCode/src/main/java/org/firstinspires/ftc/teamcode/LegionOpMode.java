package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;
@Disabled
@TeleOp(name="Slide DriveTrain", group="Driver Controlled")
public class LegionOpMode extends OpMode {
    public final double armP = 0.009;
    public final double armI = 0.001;
    public final double armD = 0.001;

    public final double flywheelOneP = 0;
    public final double flywheelOneI = 0;
    public final double flywheelOneD = 0;

    public final double flywheelTwoP = 0;
    public final double flywheelTwoI = 0;
    public final double flywheelTwoD = 0;

    private double leftPower = 0;
    private double rightPower = 0;
    private double strafePower = 0;
    private double firstFlyPower = 0;
    private double secondFlyPower = 0;
    private double wobbleMotorPower = 0;
    private double wobbleMotorPosition = 0;

    private boolean isRBDown = false;
    private boolean isLBDown = false;

    private int wobbleGrabPosition = 0;
    private int wobbleArmPositionSetpoints = 0;

    private ElapsedTime clock = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double elapsedTime = 0;

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor strafeDrive = null;
    private DcMotor wobbleArm = null;
    private DcMotor firstFlywheel = null;
    private DcMotor secondFlywheel = null;
    private Servo wobbleGrab = null;
    private PID armPID = null;
    private PID flywheelOnePID = null;
    private PID flywheelTwoPID = null;

    @Override
    public void init() {
        armPID = new PID(armP, armI, armD, 0);
        flywheelOnePID = new PID(flywheelOneP, flywheelOneI, flywheelOneD, 0);
        flywheelTwoPID = new PID(flywheelTwoP, flywheelTwoI, flywheelTwoD, 0);

        wobbleGrabPosition = 0;
        leftPower = 0;
        rightPower = 0;
        strafePower = 0;
        firstFlyPower = 0;
        secondFlyPower = 0;
        wobbleMotorPower = 0;
        wobbleMotorPosition = 0;
        isRBDown = false;
        isLBDown = false;
        wobbleGrabPosition = 0;
        wobbleArmPositionSetpoints = 0;
        elapsedTime = 0;

        leftDrive = hardwareMap.get(DcMotor.class, "lMotor");
        rightDrive = hardwareMap.get(DcMotor.class, "rMotor");
        strafeDrive = hardwareMap.get(DcMotor.class, "sMotor");
        wobbleArm = hardwareMap.get(DcMotor.class, "waMotor");
        wobbleGrab = hardwareMap.get(Servo.class, "wgServo");
        firstFlywheel = hardwareMap.get(DcMotor.class, "firstFlywheel");
        secondFlywheel = hardwareMap.get(DcMotor.class, "intakeMotor");
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafeDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobbleArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        firstFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        secondFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wobbleGrab.setPosition(0);
    }

    @Override
    public void loop() {
        resetClock();
        controls();
        setPowers();
    }

    private void resetClock() {
        elapsedTime = (double)clock.time(TimeUnit.MILLISECONDS) / 1000.0;
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
        flywheelControl();
    }

    private void setPowers() {
        leftDrive.setPower(-leftPower);
        rightDrive.setPower(-rightPower);
        strafeDrive.setPower(strafePower);
        firstFlywheel.setPower(firstFlyPower);
        secondFlywheel.setPower(secondFlyPower);
    }

    private void slideDriveControl() {
        leftPower = gamepad1.left_stick_y;
        rightPower = -gamepad1.left_stick_y;
        leftPower -= gamepad1.right_stick_x;
        rightPower -= gamepad1.right_stick_x;
        strafePower = gamepad1.left_stick_x;
    }

    private void wobbleArmControl() {
        armPID.setSetPoint(wobbleMotorPosition);
        wobbleMotorPower = armPID.PIDLoop((double)wobbleArm.getCurrentPosition());
        telemetry.addData("Flywheel 1 Speed: ", 0);
        telemetry.addData("Flywheel 2 Speed: ", 0);
        wobbleArm.setPower(wobbleMotorPower);
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

        if (gamepad1.left_bumper) {
            if (!isLBDown) {
                //gets run only once when first pressed
                wobbleArmPositionSetpoints++;
                if (wobbleArmPositionSetpoints > 2) {
                    wobbleArmPositionSetpoints = 0;
                }
            }
            isLBDown = true;
        } else {
            isLBDown = false;
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

        switch (wobbleArmPositionSetpoints) {
            case 1:
                wobbleMotorPosition = -280;
                break;
            case 2:
                wobbleMotorPosition = -110;
                break;
            case 0:
                wobbleMotorPosition += (gamepad1.left_trigger - gamepad1.right_trigger) * elapsedTime * 100;
                break;
            default:
                wobbleMotorPosition = (gamepad1.left_trigger - gamepad1.right_trigger - 0.5) * 200;
                break;
        }
    }

    private void flywheelControl() {
        if (gamepad1.y) {
            firstFlyPower = 1;
            secondFlyPower = 1;
        } else {
            firstFlyPower = 0;
            secondFlyPower = 0;
        }
    }
}
