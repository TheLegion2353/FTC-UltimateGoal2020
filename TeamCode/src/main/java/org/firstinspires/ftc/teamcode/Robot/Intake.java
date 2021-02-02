package org.firstinspires.ftc.teamcode.Robot;
import java.util.ArrayList;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Intake extends RobotPart {
	private HardwareController motorGroup = null;

	public Intake(Gamepad gp, DcMotor ... motors) {
		super(gp);
		motorGroup = new HardwareController(DcMotor.RunMode.RUN_USING_ENCODER, motors);
	}

	@Override
	protected void driverUpdate() {
		motorGroup.setSpeed(gamepad.right_trigger - gamepad.left_trigger);
	}

	@Override
	protected void autonomousUpdate() {

	}
}
