package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Drive Via Gamepad", group = "ftc16671")
public class MecanumDriveOpMode extends OpMode {
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private double[] distances;
    double pusherPosition, pusherMinPosition, pusherMaxPosition,  lifterPosition, grabberPosition, armPosition;
    double  MIN_POSITION = 0, MAX_POSITION = 1;
    double SERVO_OFFSET = 0.005;
    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        mecanumDrive.init(hardwareMap);
        mecanumDrive.initServo(hardwareMap);
        mecanumDrive.initIntakeAndConveyor(hardwareMap);
        mecanumDrive.initShooterMotors(hardwareMap);
        mecanumDrive.lifter.setPosition(1.0);
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        //setup gamepads
        //getting gamepad1 controls

        double forward = gamepad1.left_stick_y * -1; //The y direction on the gamepad is reversed idk why
        double strafe = gamepad1.left_stick_x * -1;
        double rotate = gamepad1.right_stick_x * -1;
        if (gamepad1.left_bumper){
            //if the left bumper is pressed, the speed will be reduced to 50%
            double decreaseSpeed = 0.5;
             forward = (gamepad1.left_stick_y * decreaseSpeed) * -1; //The y direction on the gamepad is reversed idk why
             strafe = (gamepad1.left_stick_x * decreaseSpeed) * -1;
             rotate = (gamepad1.right_stick_x * decreaseSpeed) * -1;
        } else if(gamepad1.left_bumper && gamepad1.right_bumper){
            double decreaseSpeed = 0.25;
            forward = (gamepad1.left_stick_y * decreaseSpeed) * -1; //The y direction on the gamepad is reversed idk why
            strafe = (gamepad1.left_stick_x * decreaseSpeed) * -1;
            rotate = (gamepad1.right_stick_x * decreaseSpeed) * -1;
        } else {
             forward = gamepad1.left_stick_y * -1; //The y direction on the gamepad is reversed idk why
             strafe = gamepad1.left_stick_x * -1;
             rotate = gamepad1.right_stick_x * -1;
        }

        /*
        double gamepadrt = gamepad1.right_trigger;
        double gamepadlt = gamepad1.left_trigger;
        boolean gamepadLeftBumper = gamepad1.left_bumper;
        boolean gamepadRightBumper = gamepad1.right_bumper;
        //getting gamepad 2 controls
        boolean gamepad2X = gamepad2.x;
        boolean gamepad2Y = gamepad2.y;
        boolean gamepad2A = gamepad2.a;
        boolean gamepad2B = gamepad2.b;
        double gamepad2rt = gamepad2.right_trigger;
        double gamepad2lt = gamepad2.left_trigger;
        boolean gamepad2LeftBumper = gamepad2.left_bumper;
        boolean gamepad2RightBumper = gamepad2.right_bumper;

         */
        //when motor starts moving, lifter position should be at bottom level
        if(mecanumDrive.frontLeft.isBusy() && mecanumDrive.backRight.isBusy()){
            mecanumDrive.lifter.setPosition(1.0);
        }

        if (gamepad1.right_trigger > 0){
            lifterPosition = .599;
            mecanumDrive.lifter.setPosition(Range.clip(lifterPosition, MIN_POSITION, MAX_POSITION));
            mecanumDrive.runIntake(-1.0);//take it in
            mecanumDrive.runConveyor(-1.0);//take it in
        }

        if (gamepad1.left_trigger > 0){
            lifterPosition = .599;
            mecanumDrive.lifter.setPosition(Range.clip(lifterPosition, MIN_POSITION, MAX_POSITION));
            mecanumDrive.runIntake(1.0);//take it out
            mecanumDrive.runConveyor(1.0);//take it out
        }

        if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0){
            mecanumDrive.runIntake(0.0);//take it out
            mecanumDrive.runConveyor(0.0);//take it out
        }


        // if gamepad 2 right trigger or left trigger are pushed even slightly,
        // both shooters will run full speed
        if(gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0){
            //mecanumDrive.shooterFront.setTargetPosition(4000000);
            //mecanumDrive.shooterBack.setTargetPosition(4000000);
            mecanumDrive.shooterFront.setPower(1);
            mecanumDrive.shooterBack.setPower(0.65);
            telemetry.addData("shooter front", "position=" +  mecanumDrive.shooterFront.getCurrentPosition());
            telemetry.addData("shooter back", "position=" +  mecanumDrive.shooterBack.getCurrentPosition());
           /* mecanumDrive.runShooterFront(1);
            mecanumDrive.runShooterBack(1); */
        }

        if(gamepad2.right_trigger == 0 && gamepad2.left_trigger == 0) {
            //mecanumDrive.shooterFront.setPower(-1);
            //mecanumDrive.shooterBack.setPower(-1);
            mecanumDrive.runShooterFront(-0.2);
            mecanumDrive.runShooterBack(-0.2);
        }

        //use gamepad2 left bumper to grab the wobber and move the arm up as well
        if(gamepad2.left_bumper){
            grabberPosition = 0.4;

            mecanumDrive.grabber.setPosition(Range.clip(grabberPosition, MIN_POSITION, MAX_POSITION));
            //mecanumDrive.wobbleArm.setPosition(Range.clip(armPosition, MIN_POSITION, MAX_POSITION));
            telemetry.addData("grabber servo", "position=" + grabberPosition + "  actual="
                    + mecanumDrive.grabber.getPosition());
            //telemetry.addData("arm servo", "position=" + armPosition + "  actual="
            //        + mecanumDrive.wobbleArm.getPosition());
        }
        if(gamepad2.right_bumper){
            grabberPosition = MIN_POSITION;

            mecanumDrive.grabber.setPosition(Range.clip(grabberPosition, MIN_POSITION, MAX_POSITION));
            //mecanumDrive.wobbleArm.setPosition(Range.clip(armPosition, MIN_POSITION, MAX_POSITION));
            telemetry.addData("grabber servo", "position=" + grabberPosition + "  actual="
                    + mecanumDrive.grabber.getPosition());
            //telemetry.addData("arm servo", "position=" + armPosition + "  actual="
            //      + mecanumDrive.wobbleArm.getPosition());
        }

        if(gamepad2.dpad_up){
            armPosition = .5;
            mecanumDrive.wobbleArm.setPosition(Range.clip(armPosition, MIN_POSITION, MAX_POSITION));
            telemetry.addData("arm servo", "position=" + armPosition + "  actual="
                    + mecanumDrive.wobbleArm.getPosition());
        }
        if(gamepad2.dpad_down){
            armPosition = .33;
            mecanumDrive.wobbleArm.setPosition(Range.clip(armPosition, MIN_POSITION, MAX_POSITION));
            telemetry.addData("arm servo", "position=" + armPosition + "  actual="
                    + mecanumDrive.wobbleArm.getPosition());
        }

        /*
        //use gamepad2 right bumper to move the arm down and release the wobble.
        if(gamepad2.right_bumper){
            if(armPosition < MAX_POSITION){
                armPosition = MAX_POSITION;
            }
            if(armPosition == MAX_POSITION){
                armPosition = MIN_POSITION;
            }
        }
        mecanumDrive.grabber.setPosition(Range.clip(grabberPosition, MIN_POSITION, MAX_POSITION));
        telemetry.addData("wobber_arm servo", "position=" + grabberPosition + "  actual="
                + mecanumDrive.grabber.getPosition());

        */
        //Setting gamepad2B***************************
/*pusherMinPosition = 0.1;
pusherMaxPosition = 0.6;
        if (gamepad2.b) {
            if (pusherPosition > pusherMinPosition) {
                pusherPosition = pusherMinPosition;

             }
            mecanumDrive.pusher.setPosition(Range.clip(pusherPosition, pusherMinPosition, pusherMaxPosition));
            //wait
            MecanumDrive.sleep(500);
            //
            if(mecanumDrive.pusher.getPosition() == pusherMinPosition ){
                pusherPosition =  pusherMaxPosition;
            }

            mecanumDrive.pusher.setPosition(Range.clip(pusherPosition, pusherMinPosition, pusherMaxPosition));
            telemetry.addData("pusher servo", "position=" + pusherPosition + "  actual="
                    + mecanumDrive.pusher.getPosition());
        }
        */
        pusherPosition =0;
        pusherMinPosition = 0;
        pusherMaxPosition = 0.59;
        mecanumDrive.pusher.setPosition(pusherMaxPosition);

        if (gamepad2.b) {
            if (pusherPosition > pusherMinPosition) {
                pusherPosition = pusherMinPosition;

            }
            //mecanumDrive.pusher.setPosition(Range.clip(pusherPosition, pusherMinPosition, pusherMaxPosition));
            mecanumDrive.pusher.setPosition(pusherMinPosition);
            telemetry.addData("pusher servo", "position=" + pusherPosition + "  actual="
                    + mecanumDrive.pusher.getPosition());
            //wait
            MecanumDrive.sleep(700);//400
            //
            if(mecanumDrive.pusher.getPosition() <= pusherMinPosition ){
                pusherPosition =  pusherMaxPosition;
            }
            //mecanumDrive.pusher.setPosition(Range.clip(pusherPosition, pusherMinPosition, pusherMaxPosition));
            mecanumDrive.pusher.setPosition(pusherMaxPosition);

            telemetry.addData("pusher servo", "position=" + pusherPosition + "  actual="
                    + mecanumDrive.pusher.getPosition());
        }





        //Setting gamepad2Y***************************
        if(gamepad2.y){
            if(lifterPosition > 0.2) {
                lifterPosition -= SERVO_OFFSET;
            }
            mecanumDrive.lifter.setPosition(Range.clip(lifterPosition, MIN_POSITION, MAX_POSITION));
            telemetry.addData("lifter servo", "position=" + lifterPosition + "  actual="
                    + mecanumDrive.lifter.getPosition());
        }


/*
        //Setting gamepad2A***************************
        if(gamepad2.a){
            if(lifterPosition < 0.8){
                lifterPosition += SERVO_OFFSET;
            }
            mecanumDrive.lifter.setPosition(Range.clip(lifterPosition, MIN_POSITION, MAX_POSITION));
            telemetry.addData("lifter servo", "position=" + lifterPosition + "  actual="
                    + mecanumDrive.lifter.getPosition());

        }
*/
        if(gamepad2.a){
            lifterPosition = 0.503;
            mecanumDrive.lifter.setPosition(Range.clip(lifterPosition, MIN_POSITION, MAX_POSITION));
            telemetry.addData("lifter servo", "position=" + lifterPosition + "  actual="
                    + mecanumDrive.lifter.getPosition());

        }
        if(gamepad2.x){
            lifterPosition = .599;
            mecanumDrive.lifter.setPosition(Range.clip(lifterPosition, MIN_POSITION, MAX_POSITION));
            telemetry.addData("lifter servo", "position=" + lifterPosition + "  actual="
                    + mecanumDrive.lifter.getPosition());

        }
        if(gamepad2.y){
            lifterPosition = .510;//.515,.517,.519,.523
            mecanumDrive.lifter.setPosition(Range.clip(lifterPosition, MIN_POSITION, MAX_POSITION));
            telemetry.addData("lifter servo", "position=" + lifterPosition + "  actual="
                    + mecanumDrive.lifter.getPosition());

        }
        //supply gamepad values to run motors, servo and other parts of robots
        mecanumDrive.driveMecanum(forward, strafe, rotate);
        distances = mecanumDrive.getDistanceCm();
        telemetry.addData("distance fwd", distances[0]);
        telemetry.addData("distance right", distances[1]);
        telemetry.update();



    }

    /**
     *
     * @param miliseconds
     */
    private void justWait(int miliseconds){

        double currTime = getRuntime();
        double waitUntil = currTime + (double)(miliseconds/1000);
        while (getRuntime() < waitUntil){
            //do nothing
        }

    }


}