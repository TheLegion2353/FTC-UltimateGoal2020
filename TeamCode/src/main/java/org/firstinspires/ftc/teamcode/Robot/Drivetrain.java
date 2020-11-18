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
	double xPosition = 0;
	double yPosition = 0;
	double targetX = 0;
	double targetY = 0;
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

	public double[] getPosition() {
		return new double[] {xPosition, yPosition, (angle + 0.5)};
	}

	public void move(double x, double y) {
		targetX = x;
		targetY = y;
	}

	public void moveAngle(double a) {
		targetAngle = a;
	}

	public void move(double x, double y, double a) {
		targetX = x;
		targetY = y;
		targetAngle = a;
	}

	public void setPosition(double x, double y, double a) {
		xPosition = x;
		yPosition = y;
		angle = a;
	}

	@Override
	protected void autonomousUpdate() {
		//BEFORE DECLARING THIS CODE TO NOT WORK, REMEMBER TO TEST THE PID VALUES!
		//relying on navigation targets for now.  Will add independent odometry later.
		double referenceAngle = angleToTarget() - angle;
		double deltax = targetX - xPosition; //temporarily store the x and y components to find the distance.
		double deltay = targetY - yPosition;
		double distance = Math.pow(Math.pow(deltax, 2) + Math.pow(deltay, 2), 1/2);
		deltax = Math.sin(referenceAngle)*distance;
		deltay = Math.cos(referenceAngle)*distance;
		//basically, these lines above compose a x and y value that needs to be moved relative to the current angle and the current position as well as the current angle.


		//---ANGLE---
		//This block of code controls the angle and is independent of the position block of code (below this block).  This code isn't really efficient.
		if (Math.abs(referenceAngle) < 1) {
			anglePIDController.setSetPoint(angleToTarget());
			double anglePower = anglePIDController.PIDLoop(angle);
			leftGroup.setSpeed(anglePower); //these two lines are overwritten by the next block of code.
			rightGroup.setSpeed(-anglePower);
		}


		//---POSITION---
		//getting the position right;  This block of code controls the position and is independent of the angle block of code (above this block).
		xPIDController.setSetPoint(0); //the process variable is delta x and delta y, so this needs to be zero as we want to get close to the target position
		yPIDController.setSetPoint(0);
		double xPower = xPIDController.PIDLoop(deltax);
		double yPower = yPIDController.PIDLoop(deltay);
		leftGroup.setSpeed(yPower);
		rightGroup.setSpeed(yPower);
		centerGroup.setSpeed(xPower);
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
		double dx = xPosition - targetX;
		double dy = yPosition - targetY;
		return Math.toDegrees(Math.atan2(dy, dx)) + 90;
	}

	public enum ControlType {
		TANK,
		ARCADE
	}
}

