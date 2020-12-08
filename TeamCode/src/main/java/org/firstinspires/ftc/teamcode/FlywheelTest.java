package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name="Flywheel Test", group="Driver Controlled")
public class FlywheelTest extends OpMode {

	DcMotor flywheel = null;
	@Override
	public void init() {
		flywheel = hardwareMap.get(DcMotor.class, "motor1");
		flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}

	@Override
	public void loop() {
		if (gamepad1.y) {
			flywheel.setPower(1);
		} else {
			flywheel.setPower(0);
		}
	}
}
