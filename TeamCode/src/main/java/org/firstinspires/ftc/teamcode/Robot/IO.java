package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.Gamepad;

public class IO {
	private Gamepad gamepad = null;
	private controlType control = null;

	public IO(Gamepad gp, controlType ct) {
		gamepad = gp;
		control = ct;
	}

	public Gamepad getGamepad() {
		return gamepad;
	}

	public double getDouble() {
		return 0d;
	}

	public boolean getBool() {
		return true;
	}

	public enum controlType {
		DOUBLE,
		BOOLEAN
	}

	public enum doubleInputs {
		RIGHTX,
		RIGHTY,
		LEFTX,
		LEFTY,
		LEFTTRIGGER,
		RIGHTTRIGGER
	}

	public enum booleanInputs {
		UP,
		DOWN,
		LEFT,
		RIGHT,
		LEFTBUMPER,
		RIGHTBUMPER,
		A,
		B,
		Y,
		X,
		BACK,
		START
	}
}
