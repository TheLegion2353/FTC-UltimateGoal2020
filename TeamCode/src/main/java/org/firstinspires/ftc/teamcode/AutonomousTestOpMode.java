package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name="Test Auto", group="Autonomous")
public class AutonomousTestOpMode extends OpMode {
    Robot robot = null;
    boolean isDone = false;
    @Override
    public void init() {
        robot = new Robot(gamepad1, telemetry);
        isDone = false;
        robot.setLeftGroup(hardwareMap.get(DcMotor.class, "lMotor"));
        robot.setRightGroup(hardwareMap.get(DcMotor.class, "rMotor"));
        robot.setSlideGroup(hardwareMap.get(DcMotor.class, "sMotor"));
        robot.setArmMotor(hardwareMap.get(DcMotor.class, "waMotor"));
        robot.setShooter(hardwareMap.get(DcMotor.class, "firstFlywheel"));
        robot.setIntake(hardwareMap.get(DcMotor.class, "intakeMotor"));
        robot.setGrabber(hardwareMap.get(Servo.class, "wgServo"));
        robot.addElevator(hardwareMap.get(CRServo.class, "elevatorServo"));
    }


    @Override
    public void loop() {
        while (!isDone) {
            robot.move(0, 100);
            isDone = true;
        }
    }
}