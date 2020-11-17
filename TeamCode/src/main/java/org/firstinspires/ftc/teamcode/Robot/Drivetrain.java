package org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.PID;


public class Drivetrain extends RobotPart {
	private ControlType control;
	private HardwareController leftGroup = null;
	private HardwareController rightGroup = null;
	private HardwareController centerGroup = null;
	private PID anglePIDController = null;
	private PID xPIDController = null;
	private PID yPIDController = null;
	int xPosition = 0;
	int yPosition = 0;
	int targetX = 0;
	int targetY = 0;
	double angle = 0;
	double targetAngle = 0;

	public Drivetrain(ControlType ct, Gamepad gp) {
		super(gp);
		anglePIDController = new PID(1, 1, 1,0);
		xPIDController = new PID(1, 1, 1, 0);
		yPIDController = new PID(1, 1, 1, 0);
		control = ct;
		leftGroup = new HardwareController();
		rightGroup = new HardwareController();
		centerGroup = new HardwareController();
	}

	public void setLeftGroup(DcMotor.RunMode mode, DcMotor ... motors) {
		for (DcMotor m : motors) {
			leftGroup = new HardwareController(mode, motors);
		}
	}

	public void setRightGroup(DcMotor.RunMode mode, DcMotor ... motors) {
		for (DcMotor m : motors) {
			rightGroup = new HardwareController(mode, motors);
		}
	}

	public void setSlideGroup(DcMotor.RunMode mode, DcMotor ... motors) {
		for (DcMotor m : motors) {
			centerGroup = new HardwareController(mode, motors);
		}
	}

	public int[] getPosition() {
		return new int[] {xPosition, yPosition, (int)(angle + 0.5)};
	}

	public void move(int x, int y) {
		targetX = x;
		targetY = y;
	}

	public void moveAngle(double a) {
		targetAngle = a;
	}

	public void move(int x, int y, double a) {
		targetX = x;
		targetY = y;
		targetAngle = a;
	}

	public void setPosition(int x, int y, double a) {
		xPosition = x;
		yPosition = y;
		angle = a;
	}

	@Override
	protected void autonomousUpdate() {
		double referenceAngle = angleToTarget() - angle;
		double deltax = targetX - xPosition;
		double deltay = targetY - yPosition;
		xPosition = ;
		yPosition = ;
		if (Math.abs(referenceAngle) < 1) {
			anglePIDController.setSetPoint(angleToTarget());
			double anglePower = anglePIDController.PIDLoop(angle);
			if (anglePower > 1) {
				anglePower = 1;
			} else if (anglePower < -1) {
				anglePower = -1;
			}
			leftGroup.setSpeed(anglePower);
			rightGroup.setSpeed(-anglePower);
		} else {
			xPIDController.setSetPoint(targetX);
			yPIDController.setSetPoint(targetY);
			double xPower = xPIDController.PIDLoop(xPosition);
			double yPower = yPIDController.PIDLoop(yPosition);
			leftGroup.setSpeed((xPower + yPower) / 2);
			rightGroup.setSpeed((xPower + yPower) / 2);
		}
	}

	@Override
	protected void driverUpdate() {
		if (gamepad != null) {
			switch (control) {
				case TANK:
					leftGroup.setSpeed(gamepad.left_stick_y);
					rightGroup.setSpeed(gamepad.right_stick_y);
					break;

				case ARCADE:
					double leftPower = gamepad.left_stick_y - gamepad.right_stick_x;
					double rightPower = -gamepad.left_stick_y - gamepad.right_stick_x;
					leftPower = -leftPower;
					rightPower = -rightPower;
					double strafePower = gamepad.left_stick_x;
					if (Math.abs(strafePower) < .5f) {
						strafePower /= 5;
					} else if (strafePower >= 0) {
						strafePower = Math.pow(10, strafePower - 1.10914) + 1 - Math.pow(10, -0.10914);
					} else {
						strafePower = -(Math.pow(10, ((-strafePower) - 1.10914)) + 1 - Math.pow(10, -0.10914));
					}
					leftGroup.setSpeed(leftPower);
					rightGroup.setSpeed(rightPower);
					centerGroup.setSpeed(strafePower);
					break;
			}
		}
	}

	private double angleToTarget() {
		double deltax = xPosition - targetX;
		double deltay = yPosition - targetY;
		return Math.toDegrees(Math.atan2(deltay, deltax)) + 90;
	}

	public enum ControlType {
		TANK,
		ARCADE
	}
}

