package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RoadRunner Specific Imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Regular FTC Imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/*
  Wireless Code Download: Terminal --> "adb connect 192.168.43.1:5555"
 */


@Config
@Autonomous
public class SquareTestRoute extends LinearOpMode {

    public class Arm {
        private DcMotorEx arm;

        public Arm(HardwareMap hardwareMap) {
            arm = hardwareMap.get(DcMotorEx.class, "arm");
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setDirection(DcMotorSimple.Direction.REVERSE);

            // Reset the arm's position, then let the code control it
            arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        public class ArmOffFloor implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm.setPower(0.8);
                    initialized = true;
                }

                double position = arm.getCurrentPosition();
                packet.put("Arm Position", position);
                if (position < 500.0) {
                    return true;
                } else {
                    arm.setPower(0);
                    return false;
                }
            }
        }
        public Action ArmOffFloor() {
            return new ArmOffFloor();
        }

        public class ArmToFloor implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm.setPower(-0.8);
                    initialized = true;
                }

                double position = arm.getCurrentPosition();
                packet.put("Arm Position", position);
                if (position > 0.0) {
                    return true;
                } else {
                    arm.setPower(0);
                    return false;
                }
            }
        }
        public Action ArmToFloor(){
            return new ArmToFloor();
        }
    }


    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(90)));
        Arm arm = new Arm(hardwareMap);

        Action trajectoryAction1;
        Action trajectoryAction2;

        trajectoryAction1 = drive.actionBuilder(drive.pose)
                .lineToY(24)
                .turn(Math.toRadians(90))
                .lineToX(-24)
                .turn(Math.toRadians(90))
                .lineToY(0)
                .turn(Math.toRadians(90))
                .lineToX(0)
                .turn(Math.toRadians(90))
                .build();

        trajectoryAction2 = drive.actionBuilder(drive.pose)
                .lineToY(24)
                .strafeTo(new Vector2d(-24, 24))
                .turn(Math.toRadians(180))
                .lineToY(0)
                .turn(Math.toRadians(90))
                .lineToX(0)
                .turn(Math.toRadians(90))
                .build();


        Action trajectoryActionChosen = trajectoryAction1;

        while (!isStopRequested() && !opModeIsActive()) {
            if (gamepad1.dpad_up) {
                trajectoryActionChosen = trajectoryAction1;
            } else if (gamepad1.dpad_down) {
                trajectoryActionChosen = trajectoryAction2;
            }
        }

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        arm.ArmOffFloor(),
                        trajectoryActionChosen,
                        arm.ArmToFloor()
                )
        );
    }
}