package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.PID;

public class Arm {
    public final double armP = 0.009;
    public final double armI = 0.001;
    public final double armD = 0.001;

    private int wobbleArmPositionSetpoints = 0;
    private boolean isLBDown = false;
    private Gamepad gamepad = null;
    private DcMotor motor = null;
    private PID PIDController = null;

    public Arm(Gamepad gp, DcMotor m) {
        gamepad = gp;
        motor = m;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PIDController = new PID(armP, armI, armD, motor.getCurrentPosition());
    }

    public void update(double elapsedTime) {
        int pos = 0;
        if (gamepad.left_bumper) {
            if (!isLBDown) {
                //gets run only once when first pressed
                wobbleArmPositionSetpoints++;
                if (wobbleArmPositionSetpoints > 2) {
                    wobbleArmPositionSetpoints = 1;
                }
            }
            isLBDown = true;
        } else {
            isLBDown = false;
        }

        switch (wobbleArmPositionSetpoints) {
            case 1:
                pos = -280;
                break;
            case 2:
                pos = -110;
                break;

        }

        PIDController.setSetPoint(pos);
        double power = PIDController.PIDLoop((double)motor.getCurrentPosition(), elapsedTime);
        motor.setPower(power);
    }

    public double getArmPosition() {
        return motor.getCurrentPosition();
    }
}
