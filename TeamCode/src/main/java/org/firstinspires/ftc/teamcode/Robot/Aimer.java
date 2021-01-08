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
		armP = 0.0015;
		armI = 0.005;
		armD = 0.0015;
		PIDController = new PID(armP, armI, armD, motor.getCurrentPosition());
	}

	@Override
	protected void driverUpdate() {
		if (gamepad.dpad_left) {
			if (!isLeftPressed) {
				//gets run only once when first pressed
				aimPositionSetpoints++;
				if (aimPositionSetpoints > 3) {
					aimPositionSetpoints = 1;
				}
			}
			isLeftPressed = true;
		} else {
			isLeftPressed = false;
		}

		switch (aimPositionSetpoints) {
			case 1:
				position = 900;
				break;
			case 2:
				position = 1050;
				break;
			case 3:
				position = 100;
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

	@Override
	protected void autonomousUpdate() {
		PIDController.setSetPoint(position);
		double power = PIDController.PIDLoop((double)motor.getCurrentPosition());
		motor.setPower(power);
		if (telemetry != null) {
			telemetry.addData("Position: ", position);
			telemetry.update();
		}
	}

	public boolean setAngle(double s) {
		PIDController.setSetPoint(s);
		position = s;
		return Math.abs(motor.getCurrentPosition() - s) < 50;
	}
}
