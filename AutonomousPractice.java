package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Pinpoint;

@Autonomous(name = "Pinpoint Auto")
public class AutonomousPractice extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Change the names of the motor to what you anemd your motor
        DcMotor lf = hardwareMap.get(DcMotor.class, "Left Front Motor");
        DcMotor lr = hardwareMap.get(DcMotor.class, "Left Rear Motor");
        DcMotor rf = hardwareMap.get(DcMotor.class, "Right Front Motor");
        DcMotor rr = hardwareMap.get(DcMotor.class, "Right Rear Motor");

        // INIT PINPOINT
        Pinpoint pinpoint = new Pinpoint(hardwareMap);

        MethodHolder drive = new MethodHolder(lf, rf, lr, rr, pinpoint);

        waitForStart();

        // 🔥 RUN ONE COMMAND (change this to test)
        drive.moveForward(24);
        // drive.strafeRight(24);
        // drive.turnRight(Math.toRadians(90));

        while (opModeIsActive()) {

            pinpoint.update();   // ALWAYS update first
            drive.update();

            telemetry.addData("X", pinpoint.getX());
            telemetry.addData("Y", pinpoint.getY());
            telemetry.addData("Heading", Math.toDegrees(pinpoint.getHeading()));
            telemetry.update();
        }
    }
}
