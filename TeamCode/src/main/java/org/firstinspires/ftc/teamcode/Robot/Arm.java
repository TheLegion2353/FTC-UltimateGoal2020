package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PID;

public class Arm extends RobotPart {
    public double armP = 0.009;
    public double armI = 0.001;
    public double armD = 0.001;

    private int wobbleArmPositionSetpoints = 0;
    private boolean isLBDown = false;
    protected DcMotor motor = null;
    protected PID PIDController = null;
    protected Telemetry telemetry = null;
    protected double position = 0;

    public Arm(Gamepad gp, DcMotor m) {
        super(gp);
        motor = m;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PIDController = new PID(armP, armI, armD, motor.getCurrentPosition());
    }

    public Arm(Gamepad gp, DcMotor m, Telemetry t) {
        super(gp);
        motor = m;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PIDController = new PID(armP, armI, armD, motor.getCurrentPosition());
    }

    @Override
    protected void autonomousUpdate() {
        PIDController.setSetPoint(position);
        double power = PIDController.PIDLoop((double)motor.getCurrentPosition());
        motor.setPower(power);
    }

    @Override
    protected void driverUpdate() {
        if (gamepad.left_bumper) {
            if (!isLBDown) {
                //gets run only once when first pressed
                wobbleArmPositionSetpoints++;
                if (wobbleArmPositionSetpoints > 2) {
                    wobbleArmPositionSetpoints = 1;
                }
            }
            isLBDown = true;
        } else {
            isLBDown = false;
        }

        switch (wobbleArmPositionSetpoints) {
            case 1:
                position = 50;
                break;
            case 2:
                position = 410;
                break;

        }

        PIDController.setSetPoint(position);
        double power = PIDController.PIDLoop((double)motor.getCurrentPosition());

        if (gamepad.a) {
            power = -0.5;
        }

        if (gamepad.start) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        motor.setPower(power);
        if (telemetry != null) {
            telemetry.addData("Position: ", position);
            telemetry.update();
        }
    }

    public boolean setArmPosition(double pos) {
        position = pos;
        return Math.abs(motor.getCurrentPosition() - pos) < 50;
    }

    public double getArmPosition() {
        return motor.getCurrentPosition();
    }
}
