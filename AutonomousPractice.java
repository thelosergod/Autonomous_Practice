package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="2-Odometry-Pod")
public class AutonomousPractice extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor left_front_motor = hardwareMap.get(DcMotor.class, "Left Front Motor");
        DcMotor left_rear_motor = hardwareMap.get(DcMotor.class, "Left Rear Motor");
        DcMotor right_front_motor = hardwareMap.get(DcMotor.class, "Right Front Motor");
        DcMotor right_rear_motor = hardwareMap.get(DcMotor.class, "Right Rear Motor");

        DcMotor xOdometry = hardwareMap.get(DcMotor.class, "X-Axis OdometryPod");
        DcMotor yOdometry = hardwareMap.get(DcMotor.class, "Y-Axis OdometryPod");

        MethodHolder goForward = new MethodHolder(left_front_motor, right_front_motor, left_rear_motor, right_rear_motor, xOdometry, yOdometry,
                12, 0);

        waitForStart();

        while (opModeIsActive()) {
            goForward.update();

            telemetry.addData("X Odometry", xOdometry.getCurrentPosition());
            telemetry.addData("Y Odometry", yOdometry.getCurrentPosition());
            telemetry.update();
        }
    }
}


