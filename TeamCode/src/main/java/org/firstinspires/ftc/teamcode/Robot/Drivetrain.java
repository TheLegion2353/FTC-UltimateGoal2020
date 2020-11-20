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
	boolean hasStarted = false;
	double netPowerLeft = 0;
	double netPowerRight = 0;
	double netPowerCenter = 0;

	private Telemetry telemetry = null;


	public Drivetrain(ControlType ct, Gamepad gp, Telemetry t) {
		super(gp);
		telemetry = t;
		anglePIDController = new PID(0.01, 0, 0,0);
		xPIDController = new PID(0.00075, 0, 0.000025, 0.00001);
		yPIDController = new PID(0.00075, 0, 0.000025, 0.00001);
		control = ct;
		leftGroup = new HardwareController();
		rightGroup = new HardwareController();
		centerGroup = new HardwareController();
	}

	public Drivetrain(ControlType ct, Gamepad gp) {
		super(gp);
		anglePIDController = new PID(0, 0, 0,0);
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
		return (Math.abs(targetX - xPosition) < 100 && Math.abs(targetY - yPosition) < 100);
	}

	public boolean moveAngle(double a) {
		targetAngle = a;
		return (Math.abs(targetAngle - angle) < 100);
	}

	public boolean move(double x, double y, double a) {
		targetX = x;
		targetY = y;
		targetAngle = a;
		return (Math.abs(targetX - xPosition) < 100 && Math.abs(targetY - yPosition) < 100 && Math.abs(targetAngle - angle) < 100);
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
			if (hasStarted == false) {
			/*
			//assuming the angle remains constant.
			double deltaYPrime = (((leftGroup.getPos() - rightGroup.getPos()) / (double)2) - lastEncoderY) / (560 * 2 * 1.772 * Math.PI);
			double deltaXPrime = (centerGroup.getPos() - lastEncoderX) / (560 * 2 * 1.772 * Math.PI);
			double dist = Math.pow(Math.pow(deltaXPrime, 2) + Math.pow(deltaYPrime, 2), (double)1/2);
			double f = deltaXPrime/Math.tan(Math.toRadians(angle)) - deltaYPrime;
			double deltaY = Math.sin(Math.toRadians(angle)) * f;
			double deltaX = Math.cos(Math.toRadians(angle)) * f;
			yPosition -= deltaY;
			xPosition -= deltaX;
			telemetry.addData("DX: ", deltaX);
			telemetry.addData("DY: ", deltaY);
			*/
				netPowerLeft = 1;
				netPowerRight = -0.9;
				netPowerCenter = 0;
				hasStarted = true;
			}
		} else {
			netPowerLeft = 0;
			netPowerRight = 0;
			netPowerCenter = 0;
			double referenceAngle = angleToTarget() - angle;
			double deltax = targetX - xPosition; //temporarily store the x and y components to find the distance.
			double deltay = targetY - yPosition;
			double distance = Math.pow(Math.pow(deltax, 2) + Math.pow(deltay, 2), (double) 1 / 2);
			deltay = Math.sin(Math.toRadians(referenceAngle)) * distance; //may need to swap sine and cosine functions if the angleToTarget() function needs to return its value + 90 degrees.
			deltax = Math.cos(Math.toRadians(referenceAngle)) * distance;
			//basically, these lines above compose a x and y value that needs to be moved relative to the current angle and the current position.

			//---ANGLE---
			//This block of code controls the angle and is independent of the position block of code (below this block).

			anglePIDController.setSetPoint(targetAngle);
			double anglePower = -anglePIDController.PIDLoop(angle);
			telemetry.addData("angle: ", angle);
			telemetry.addData("angleto: ", targetAngle);
			netPowerLeft += anglePower;
			netPowerRight += anglePower;


			//---POSITION---
			//getting the position right;  This block of code controls the position and is independent of the angle block of code (above this block).
			xPIDController.setSetPoint(0); //the process variable is delta x and delta y, so this needs to be zero as we want to get close to the target position
			yPIDController.setSetPoint(0);
			double xPower = xPIDController.PIDLoop(deltax);
			double yPower = yPIDController.PIDLoop(deltay);
			netPowerLeft += yPower;
			netPowerRight -= yPower;
			netPowerCenter += xPower;
			telemetry.addData("Position X: ", xPosition);
			telemetry.addData("Position Y: ", yPosition);
			telemetry.addData("Target X: ", targetX);
			telemetry.addData("Target Y: ", targetY);
			needManualOdometry = true; //set flag to true.  If the setPosition() is called, signifying a definitive position is known, needManualOdometry will be set back to false.
			lastEncoderX = (leftGroup.getPos() - rightGroup.getPos()) / (double) 2; //subtracted because they have different signs as the motors face opposite directions.
			lastEncoderY = (centerGroup.getPos());
		}
		leftGroup.setSpeed(netPowerLeft);
		rightGroup.setSpeed(netPowerRight);
		centerGroup.setSpeed(netPowerCenter);
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

