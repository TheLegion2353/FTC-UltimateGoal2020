package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Elevator extends RobotPart {
	private HardwareController elevator = null;

	public Elevator(Gamepad gp, CRServo crs) {
		super(gp);
		elevator = new HardwareController();
		elevator.addCRServo(crs);
	}

	@Override
	protected void driverUpdate() {
		if (gamepad.x) {
			elevator.setSpeed(1);
		} else if (gamepad.a) {
			elevator.setSpeed(-1);
		} else {
			elevator.setSpeed(0);
		}
	}

	@Override
	protected void autonomousUpdate() {

	}
}
