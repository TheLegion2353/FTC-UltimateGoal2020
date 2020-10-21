package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Robot {
    private Drivetrain slide = null;
    private Arm arm = null;

    public Robot() {
        slide = new Drivetrain(Drivetrain.ControlType.ARCADE);
    }

    public void update(double leftX, double leftY, double rightX, double rightY, double armPosition, double elapsedTime) {
        slide.update(leftX, leftY, rightX, rightY);
        arm.update(armPosition, elapsedTime);
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
        arm = new Arm(m);
    }

    public double getArmPosition() {
        return arm.getArmPosition();
    }
}
