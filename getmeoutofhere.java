/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*




                   ↓  ↓  ↓  spaghetti code  ↓  ↓  ↓



*/
@TeleOp(name = "GAS GASSS GASSSSSSSSSSSSSSSSSSSSS", group = "Concept")
@Config
public class getmeoutofhere extends LinearOpMode {

    public static double POWER = 0.5;


    // Define class members
    DcMotorEx motor_1;
    DcMotorEx motor_2;
    DcMotorEx motor_3;
    DcMotorEx motor_4;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    @Override
    public void runOpMode() {
        motor_1 = hardwareMap.get(DcMotorEx.class, "topleft"); // 0
        motor_2 = hardwareMap.get(DcMotorEx.class, "topright"); // 1
        motor_3 = hardwareMap.get(DcMotorEx.class, "botleft"); // 2
        motor_4 = hardwareMap.get(DcMotorEx.class, "botright"); // 3

        // Wait for the start button
        telemetry.addData("> ", "press start pls ty" );
        telemetry.update();
        dashboardTelemetry.addData("> ","press start pls ty");
        dashboardTelemetry.update();
        waitForStart();

        motor_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        motor_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motor_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Ramp motor speeds till stop pressed.
        while(opModeIsActive()) {
//            motor_1.setVelocity(VELOCITY_1);
//            motor_2.setVelocity(VELOCITY_2);
//            if (gamepad1.left_stick_y > 0.1) {
//                motor_1.setPower(POWER);
//                motor_2.setPower(POWER);
//                motor_3.setPower(POWER);
//                motor_4.setPower(POWER);
//                telemetry.addData("> ","all running fwd");
//                dashboardTelemetry.addData("> ","all running fwd");
//            } else {
//                motor_1.setPower(0);
//                motor_2.setPower(0);
//                motor_3.setPower(0);
//                motor_4.setPower(0);
//                telemetry.addData("> ","all stopped");
//                dashboardTelemetry.addData("> ","all stopped");
//            }

            motor_1.setPower(POWER * gamepad1.left_stick_y);
            motor_2.setPower(POWER * gamepad1.left_stick_y);
            motor_3.setPower(POWER * gamepad1.left_stick_y);
            motor_4.setPower(POWER * gamepad1.left_stick_y);
            telemetry.addData("all at speed","%.2",POWER * gamepad1.left_stick_y);
            dashboardTelemetry.addData("all at speed","%.2",POWER * gamepad1.left_stick_y);

            telemetry.update();
            dashboardTelemetry.update();
        }

        // Turn off motor and signal done;
        motor_1.setPower(0);
        motor_2.setPower(0);
        motor_3.setPower(0);
        motor_4.setPower(0);
        telemetry.addData("> ", "finished");
        telemetry.update();
        dashboardTelemetry.addData("> ","finsihed");
        dashboardTelemetry.update();

    }
}
