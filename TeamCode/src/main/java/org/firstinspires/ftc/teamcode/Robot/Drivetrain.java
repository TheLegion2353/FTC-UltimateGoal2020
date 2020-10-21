package org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.lang.reflect.Array;
import java.util.ArrayList;

public class Drivetrain {
    private ControlType control;
    private HardwareController leftGroup = null;
    private HardwareController rightGroup = null;
    private HardwareController centerGroup = null;

    public Drivetrain(ControlType ct) {
        control = ct;
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

    public void update(double leftX, double leftY, double rightX, double rightY) {
        switch (control) {
            case TANK:
                leftGroup.setSpeed(leftY);
                rightGroup.setSpeed(rightY);
                break;

            case ARCADE:
                double leftPower = leftY - rightX;
                double rightPower = -leftY - rightX;
                leftPower = -leftPower;
                rightPower = -rightPower;
                double strafePower = leftX;
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

