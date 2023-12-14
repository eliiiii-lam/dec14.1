
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "GGTelearcade")
public class mainteleopcodearcade extends OpMode {

    DcMotor fL;
    DcMotor fR;
    DcMotor bL;
    DcMotor bR;

    DcMotor elbow;
    DcMotor elbow2;

    DcMotor VP;
    DcMotor VP2;

    Servo clawL;
    Servo clawR;

    CRServo wrist;

    static final double MOTOR_TICK_COUNT = 537.7;


    @Override
    public void init() {
        fL = hardwareMap.dcMotor.get("fL");
        fR = hardwareMap.dcMotor.get("fR");
        bL = hardwareMap.dcMotor.get("bL");
        bR = hardwareMap.dcMotor.get("bR");

        elbow = hardwareMap.dcMotor.get("elbow");
        elbow2 = hardwareMap.dcMotor.get("elbow2");

        VP = hardwareMap.dcMotor.get("VP");

        VP2 = hardwareMap.dcMotor.get("VP2");


        clawL = hardwareMap.servo.get("clawL");

        clawR = hardwareMap.servo.get("clawR");

        wrist = hardwareMap.crservo.get("wrist");

    }

    @Override
    public void loop() {


        if (Math.abs(-gamepad2.left_stick_y) > .1) {
            elbow.setPower(-gamepad2.left_stick_y * -0.8);
            elbow2.setPower(-gamepad2.left_stick_y * 0.8);
        } else {
            elbow.setPower(0);
            elbow2.setPower(0);
        }


        if (Math.abs(-gamepad1.left_stick_x) > .1) {
            fL.setPower(-gamepad1.left_stick_x * -1);
            bL.setPower(-gamepad1.left_stick_x * 1);
            fR.setPower(-gamepad1.left_stick_x * -1);
            bR.setPower(-gamepad1.left_stick_x * 1);
        } else {
            fL.setPower(0);
            bL.setPower(0);
            fR.setPower(0);
            fL.setPower(0);
        }

        if (Math.abs(-gamepad1.left_stick_y) > .1) {
            fL.setPower(-gamepad1.left_stick_y * 1);
            bL.setPower(-gamepad1.left_stick_y * 1);
            fR.setPower(-gamepad1.left_stick_y * -1);
            bR.setPower(-gamepad1.left_stick_y * -1);
        } else {
            fL.setPower(0);
            bL.setPower(0);
            fR.setPower(0);
            fL.setPower(0);
        }

        if (Math.abs(-gamepad1.right_stick_x) > .1) {
            fL.setPower(-gamepad1.right_stick_x * -1);
            bL.setPower(-gamepad1.right_stick_x * -1);
            fR.setPower(-gamepad1.right_stick_x * -1);
            bR.setPower(-gamepad1.right_stick_x * -1);
        } else {
            fL.setPower(0);
            bL.setPower(0);
            fR.setPower(0);
            fL.setPower(0);
        }











        if (gamepad2.a) {
            wrist.setPower(0.5);
        } else {
            wrist.setPower(0);
        }

        if (gamepad2.b) {
            wrist.setPower(-0.3);
        } else {
            wrist.setPower(0);
        }

        if (gamepad2.x) {
            elbowEncoder();
        }







        if (gamepad1.b) {
            VP.setPower(1);
            VP2.setPower(-1);
        } else {
            VP.setPower(0);
            VP2.setPower(0);
        }

        if (gamepad1.y) {
            VP.setPower(-1);
            VP2.setPower(1);
        } else {
            VP.setPower(0);
            VP2.setPower(0);
        }


        if (gamepad2.left_bumper) {
            clawL.setPosition(0.4);
        } else {
            clawL.setPosition(0);
        }

        if (gamepad2.right_bumper) {
            clawR.setPosition(0);
        } else {
            clawR.setPosition(0.4);
        }






        if (gamepad1.left_bumper) {
            fL.setPower(-1);
            bL.setPower(1);
            fR.setPower(-1);
            bR.setPower(1);
        } else {
            fL.setPower(0);
            bL.setPower(0);
            fR.setPower(0);
            bR.setPower(0);

        }
        if (gamepad1.right_bumper) {
            fL.setPower(1);
            bL.setPower(-1);
            fR.setPower(1);
            bR.setPower(-1);
        } else {
            fL.setPower(0);
            bL.setPower(0);
            fR.setPower(0);
            bR.setPower(0);
        }


    }





    public void elbowEncoder() {
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        double groundPos = MOTOR_TICK_COUNT / 1.5;
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int newTarget = elbow.getTargetPosition() + (int) groundPos;
        elbow.setTargetPosition(newTarget);
        elbow.setPower(0.9);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (elbow.isBusy()) {
            telemetry.addData("Status", "Running to ground position");
            telemetry.update();
        }
        elbow.setPower(0);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Status", "Path Complete");
        telemetry.update();
    }


}