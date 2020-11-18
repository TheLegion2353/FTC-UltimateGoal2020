package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Shooter extends RobotPart{
    private HardwareController shooter = null;

    public Shooter(Gamepad gp, DcMotor ... motors) {
        super(gp);
        shooter = new HardwareController(DcMotor.RunMode.RUN_USING_ENCODER, motors);
    }

    @Override
    public void driverUpdate() {
        if (gamepad.y) {
            shooter.setSpeed(1);
        } else {
            shooter.setSpeed(0);
        }
    }

    @Override
    public void autonomousUpdate() {

    }
}
