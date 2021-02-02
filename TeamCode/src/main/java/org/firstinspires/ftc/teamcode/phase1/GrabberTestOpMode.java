package org.firstinspires.ftc.teamcode.phase1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "GrabberTest", group = "ftc16671")
@Disabled
public class GrabberTestOpMode extends LinearOpMode {
    Servo grabber;

    @Override
    public void runOpMode() throws InterruptedException {
        grabber = hardwareMap.get(Servo.class, "left_hand");
        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            grabber.setPosition(1.0);
            telemetry.addData("--", "wait for two seconds.");
            telemetry.update();
            sleep(2000);
            grabber.setPosition(0.0);
        }
    }
}
