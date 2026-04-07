package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Pinpoint Auto")
public class AutonomousPractice extends LinearOpMode {

    private Pinpoint pinpoint;
    private MethodHolder drive;

    @Override
    public void runOpMode() {
        DcMotor lf = hardwareMap.get(DcMotor.class, "Left Front Motor");
        DcMotor lr = hardwareMap.get(DcMotor.class, "Left Rear Motor");
        DcMotor rf = hardwareMap.get(DcMotor.class, "Right Front Motor");
        DcMotor rr = hardwareMap.get(DcMotor.class, "Right Rear Motor");

        pinpoint = new Pinpoint(hardwareMap);
        drive = new MethodHolder(lf, rf, lr, rr, pinpoint);

        waitForStart();

        // 🔥 ADD/CHANGE COMMANDS HERE — each one runs fully before the next starts
        drive.moveForward(24);
        runUntilDone();

        // drive.strafeRight(12);
        // runUntilDone();

        // drive.turnRight(Math.toRadians(90));
        // runUntilDone();
    }

    // Runs the drive loop until the current command is complete, then stops motors
    private void runUntilDone() {
        while (opModeIsActive() && drive.isBusy()) {
            pinpoint.update();
            drive.update();
            telemetry.addData("X", pinpoint.getX());
            telemetry.addData("Y", pinpoint.getY());
            telemetry.addData("Heading", Math.toDegrees(pinpoint.getHeading()));
            telemetry.update();
        }
        drive.stop();
    }
}
