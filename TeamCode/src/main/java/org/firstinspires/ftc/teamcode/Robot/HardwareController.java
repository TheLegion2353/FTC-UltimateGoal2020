package org.firstinspires.ftc.teamcode.Robot;
import java.util.ArrayList;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareController {
	ArrayList<DcMotor> motors = null;
	ArrayList<Servo> servos = null;
	ArrayList<CRServo> crservos = null;

	public HardwareController() {
		motors = new ArrayList<DcMotor>();
		servos = new ArrayList<Servo>();
		crservos = new ArrayList<CRServo>();
	}

	public HardwareController(DcMotor.RunMode mode, DcMotor ... motorArgs) {
		motors = new ArrayList<DcMotor>();
		servos = new ArrayList<Servo>();
		crservos = new ArrayList<CRServo>();
		for (DcMotor mot : motorArgs) {
			addMotor(mot, mode);
		}
	}

	public void addMotor(DcMotor motor, DcMotor.RunMode mode) {
		motor.setMode(mode);
		motors.add(motor);
	}

	public void setSpeed(double s) {
		for (DcMotor m : motors) {
			m.setPower(s);
		}

		for (CRServo cr : crservos) {
			cr.setPower(s);
		}
	}

	public void setPosition(double p) {
		for (Servo s : servos) {
			s.setPosition(p);
		}
	}

	public void addMotor(DcMotor motor) {
		motors.add(motor);
	}

	public void addServo(Servo servo) {
		servos.add(servo);
	}

	public void addCRServo(CRServo crservo) {
		crservos.add(crservo);
	}
}
