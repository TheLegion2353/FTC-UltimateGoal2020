package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PID;

public class Aimer extends Arm {

	private boolean isLeftPressed = false;
	private int aimPositionSetpoints = 0;

	public Aimer(Gamepad gp, DcMotor m) {
		super(gp, m);
		armP = 0.00;
		armI = 0.00;
		armD = 0.00;
		PIDController = new PID(armP, armI, armD, motor.getCurrentPosition());
	}

	public Aimer(Gamepad gp, DcMotor m, Telemetry t) {
		super(gp, m, t);
		armP = 0.03;
		armI = 0.01;
		armD = 0.01;
		PIDController = new PID(armP, armI, armD, motor.getCurrentPosition());
	}

	@Override
	protected void driverUpdate() {
		if (gamepad.dpad_left) {
			if (!isLeftPressed) {
				//gets run only once when first pressed
				aimPositionSetpoints++;
				if (aimPositionSetpoints > 2) {
					aimPositionSetpoints = 1;
				}
			}
			isLeftPressed = true;
		} else {
			isLeftPressed = false;
		}

		switch (aimPositionSetpoints) {
			case 1:
				position = 34;
				break;
			case 2:
				position = 10;
				break;
		}

		PIDController.setSetPoint(position);
		double power = PIDController.PIDLoop((double)motor.getCurrentPosition());
		motor.setPower(power);
		if (telemetry != null) {
			telemetry.addData("Position: ", position);
			telemetry.update();
		}
	}
}
