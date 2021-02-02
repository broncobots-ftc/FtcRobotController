package org.firstinspires.ftc.teamcode.phase1;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Robot Op Mode", group = "ftc16671")
@Disabled
public class RobotOpMode extends OpMode {





    MecanumDrive mDrive = new MecanumDrive();

    @Override
    public void init() {
        mDrive.init(hardwareMap);
    }

    @Override
    public void loop() {
        double stickPower = gamepad1.left_stick_y;
        //
    }
}
