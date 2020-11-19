package org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PID;


public class Drivetrain extends RobotPart {
	private ControlType control;
	private HardwareController leftGroup = null;
	private HardwareController rightGroup = null;
	private HardwareController centerGroup = null;
	private PID anglePIDController = null;
	private PID xPIDController = null;
	private PID yPIDController = null;
	private double xPosition = 0;
	private double yPosition = 0;
	private double targetX = 0;
	private double targetY = 0;
	private double angle = 0;
	private double targetAngle = 0;
	private boolean needManualOdometry = true;
	double lastEncoderX = 0;
	double lastEncoderY = 0;

	private Telemetry telemetry = null;


	public Drivetrain(ControlType ct, Gamepad gp, Telemetry t) {
		super(gp);
		telemetry = t;
		anglePIDController = new PID(0.001, 0, 0,0);
		xPIDController = new PID(0.001, 0, 0, 0);
		yPIDController = new PID(0.001, 0, 0, 0);
		control = ct;
		leftGroup = new HardwareController();
		rightGroup = new HardwareController();
		centerGroup = new HardwareController();
	}

	public Drivetrain(ControlType ct, Gamepad gp) {
		super(gp);
		anglePIDController = new PID(0.001, 0, 0,0);
		xPIDController = new PID(0.001, 0, 0, 0);
		yPIDController = new PID(0.001, 0, 0, 0);
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

	public boolean move(double x, double y) {
		targetX = x;
		targetY = y;
		return (Math.abs(targetX - xPosition) < 1 && Math.abs(targetY - yPosition) < 1);
	}

	public boolean moveAngle(double a) {
		targetAngle = a;
		return (Math.abs(targetAngle - angle) < 1);
	}

	public boolean move(double x, double y, double a) {
		targetX = x;
		targetY = y;
		targetAngle = a;
		return (Math.abs(targetX - xPosition) < 1 && Math.abs(targetY - yPosition) < 1 && Math.abs(targetAngle - angle) < 1);
	}

	public void setPosition(double x, double y, double a) {
		xPosition = x;
		yPosition = y;
		angle = a;
		needManualOdometry = false; //will set to false so the odometry calc doesn't get run.
	}

	@Override
	protected void autonomousUpdate() {
		//relying on navigation targets for now.  Will add independent odometry later.
		if (needManualOdometry) {
			//assuming the angle remains constant.
			double deltaYPrime = ((leftGroup.getPos() - rightGroup.getPos()) / (double)2) - lastEncoderY;
			double deltaXPrime = centerGroup.getPos() - lastEncoderX;
			double dist = Math.pow(Math.pow(deltaXPrime, 2) + Math.pow(deltaYPrime, 2), (double)1/2);
			double angle = 0;
			double deltaY = Math.sin(Math.toRadians(angle)) * dist;
			double deltaX = Math.cos(Math.toRadians(angle)) * dist;
			yPosition += deltaY;
			xPosition += deltaX;
		}
		double referenceAngle = angleToTarget() - angle;
		double deltax = targetX - xPosition; //temporarily store the x and y components to find the distance.
		double deltay = targetY - yPosition;
		double distance = Math.pow(Math.pow(deltax, 2) + Math.pow(deltay, 2), (double)1/2);
		deltay = Math.sin(Math.toRadians(referenceAngle)) * distance; //may need to swap sine and cosine functions if the angleToTarget() function needs to return its value + 90 degrees.
		deltax = Math.cos(Math.toRadians(referenceAngle)) * distance;
		//basically, these lines above compose a x and y value that needs to be moved relative to the current angle and the current position.

		//---ANGLE---
		//This block of code controls the angle and is independent of the position block of code (below this block).  This code isn't really efficient.
		/*if (Math.abs(referenceAngle) < 1) {
			anglePIDController.setSetPoint(angleToTarget());
			double anglePower = anglePIDController.PIDLoop(angle);
			leftGroup.setSpeed(anglePower); //these two lines are overwritten by the next block of code.
			rightGroup.setSpeed(-anglePower);
		}*/

		//---POSITION---
		//getting the position right;  This block of code controls the position and is independent of the angle block of code (above this block).
		xPIDController.setSetPoint(0); //the process variable is delta x and delta y, so this needs to be zero as we want to get close to the target position
		yPIDController.setSetPoint(0);
		double xPower = xPIDController.PIDLoop(deltax);
		double yPower = yPIDController.PIDLoop(deltay);
		leftGroup.setSpeed(yPower);
		rightGroup.setSpeed(-yPower);
		centerGroup.setSpeed(xPower);
		telemetry.addData("Process Var x: ", xPower);
		telemetry.addData("delta x: ", deltax);
		needManualOdometry = true; //set flag to true.  If the setPosition() is called, signifying a definitive position is known, needManualOdometry will be set back to false.
		lastEncoderX = (leftGroup.getPos() - rightGroup.getPos()) / (double)2; //subtracted because they have different signs as the motors face opposite directions.
		lastEncoderY = (centerGroup.getPos());
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
		double dx = targetX - xPosition;
		double dy = targetY - yPosition;
		return Math.toDegrees(Math.atan2(dy, dx));
	}

	public enum ControlType {
		TANK,
		ARCADE
	}
}

