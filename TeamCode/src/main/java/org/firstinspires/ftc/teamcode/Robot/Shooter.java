package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Shooter {
    private HardwareController shooters = null;
    public Shooter(DcMotor ... motors) {
        shooters = new HardwareController(DcMotor.RunMode.RUN_USING_ENCODER, motors);
    }

    public void update(boolean spin) {

    }
}
