package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Shooter {
    private HardwareController shooter1 = null;
    private HardwareController shooter2 = null;
    Gamepad gamepad = null;
    public Shooter(Gamepad gp, DcMotor ... motors) {
        shooter1 = new HardwareController(DcMotor.RunMode.RUN_USING_ENCODER, motors[0]);
        shooter2 = new HardwareController(DcMotor.RunMode.RUN_USING_ENCODER, motors[1]);
        gamepad = gp;
    }

    public void update() {
        if (gamepad.y) {
            shooter1.setSpeed(-1);
            shooter2.setSpeed(1);
        } else {
            shooter1.setSpeed(0);
            shooter2.setSpeed(0);
        }
    }
}
