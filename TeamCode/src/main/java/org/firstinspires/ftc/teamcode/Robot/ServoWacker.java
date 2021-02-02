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
			wacker.setPosition(0);
		} else {
			wacker.setPosition(0.75);
		}
	}

	@Override
	protected void autonomousUpdate() {

	}

	public void setPosition(double p) {
		wacker.setPosition(p);
	}
}
