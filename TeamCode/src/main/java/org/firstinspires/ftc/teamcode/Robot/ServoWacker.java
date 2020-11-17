package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoWacker extends RobotPart {
	private HardwareController wacker = null;

	public ServoWacker(Gamepad gp, Servo ser) {
		super(gp);
		wacker = new HardwareController();
		wacker.addServo(ser);
	}

	@Override
	protected void driverUpdate() {
		if (gamepad.dpad_up) {
			wacker.setPosition(1);
		} else if (gamepad.dpad_down) {
			wacker.setPosition(-1);
		}
	}

	@Override
	protected void autonomousUpdate() {

	}
}
