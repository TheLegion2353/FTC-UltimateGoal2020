package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.PID;

public class Arm extends RobotPart {
    public final double armP = 0.009;
    public final double armI = 0.001;
    public final double armD = 0.001;

    private int wobbleArmPositionSetpoints = 0;
    private boolean isLBDown = false;
    private DcMotor motor = null;
    private PID PIDController = null;
    double position = 0;

    public Arm(Gamepad gp, DcMotor m) {
        super(gp);
        motor = m;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PIDController = new PID(armP, armI, armD, motor.getCurrentPosition());
    }

    @Override
    protected void autonomousUpdate() {

    }

    @Override
    protected void driverUpdate() {
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
                position = -90;
                break;
            case 2:
                position = -260;
                break;

        }

        PIDController.setSetPoint(position);
        double power = PIDController.PIDLoop((double)motor.getCurrentPosition());
        motor.setPower(power);
    }

    public void setArmPosition(double pos) {
        position = pos;
    }

    public double getArmPosition() {
        return motor.getCurrentPosition();
    }
}
