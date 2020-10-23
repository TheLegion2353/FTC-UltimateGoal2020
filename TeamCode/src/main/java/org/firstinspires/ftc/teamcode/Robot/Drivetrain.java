package org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;


public class Drivetrain {
    private ControlType control;
    private HardwareController leftGroup = null;
    private HardwareController rightGroup = null;
    private HardwareController centerGroup = null;
    Gamepad gamepad = null;

    public Drivetrain(ControlType ct, Gamepad gp) {
        control = ct;
        gamepad = gp;
        leftGroup = new HardwareController();
        rightGroup = new HardwareController();
        centerGroup = new HardwareController();
    }

    public void setLeftGroup(DcMotor.RunMode mode, DcMotor ... motors) {
        for (DcMotor m : motors) {
            leftGroup = new HardwareController(mode, motors);
        }
    }

    public void setRightGroup(DcMotor.RunMode mode, DcMotor ... motors) {
        for (DcMotor m : motors) {
            rightGroup = new HardwareController(mode, motors);
        }
    }

    public void setSlideGroup(DcMotor.RunMode mode, DcMotor ... motors) {
        for (DcMotor m : motors) {
            centerGroup = new HardwareController(mode, motors);
        }
    }

    public void update() {
        switch (control) {
            case TANK:
                leftGroup.setSpeed(gamepad.left_stick_y);
                rightGroup.setSpeed(gamepad.right_stick_y);
                break;

            case ARCADE:
                double leftPower = gamepad.left_stick_y - gamepad.right_stick_x;
                double rightPower = -gamepad.left_stick_y - gamepad.right_stick_x;
                leftPower = -leftPower;
                rightPower = -rightPower;
                double strafePower = gamepad.left_stick_x;
                leftGroup.setSpeed(leftPower);
                rightGroup.setSpeed(rightPower);
                centerGroup.setSpeed(strafePower);
                break;

        }
    }

    public enum ControlType {
        TANK,
        ARCADE
    }
}

