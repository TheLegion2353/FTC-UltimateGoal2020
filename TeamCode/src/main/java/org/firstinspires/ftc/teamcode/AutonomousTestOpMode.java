package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name="Test Auto", group="Autonomous")
public class AutonomousTestOpMode extends OpMode {
    Robot robot = new Robot(gamepad1);

    @Override
    public void init() {
        robot.setLeftGroup(hardwareMap.get(DcMotor.class, "lMotor"));
        robot.setRightGroup(hardwareMap.get(DcMotor.class, "rMotor"));
        robot.setSlideGroup(hardwareMap.get(DcMotor.class, "sMotor"));
        robot.setArmMotor(hardwareMap.get(DcMotor.class, "waMotor"));
        robot.setShooter(hardwareMap.get(DcMotor.class, "firstFlywheel"), hardwareMap.get(DcMotor.class, "secondFlywheel"));
    }


    @Override
    public void loop() {
        telemetry.update();
    }
}