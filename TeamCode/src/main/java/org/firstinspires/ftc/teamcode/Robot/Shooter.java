package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Shooter {
    private HardwareController shooter = null;
    private Gamepad gamepad = null;

    public Shooter(Gamepad gp, DcMotor ... motors) {
        shooter = new HardwareController(DcMotor.RunMode.RUN_USING_ENCODER, motors);
        gamepad = gp;
    }

    public void update() {
        if (gamepad.y) {
            shooter.setSpeed(1);
        } else {
            shooter.setSpeed(0);
        }
    }
}
