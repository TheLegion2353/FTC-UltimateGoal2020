package org.firstinspires.ftc.teamcode.Robot;
import java.util.ArrayList;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Intake {
	private HardwareController intake = null;
	private Gamepad gamepad = null;

	public Intake(Gamepad gp, DcMotor ... motors) {
		intake = new HardwareController(DcMotor.RunMode.RUN_USING_ENCODER, motors);
		gamepad = gp;
	}

	public void update() {
		intake.setSpeed(gamepad.right_trigger - gamepad.left_trigger);
	}
}
