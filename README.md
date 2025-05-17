# Power_Limit-Documentation

motor power to current sensor voltage to chassis power
motor power -> voltage -> chassis power
Acceleration limiting 

## Limit Acceleration Function
```c++
float ChassisSubsystem::limitAcceleration(float desiredRPM, float previousRPM, int power)
{
    float maxAccel = 100;
    float diff = desiredRPM - previousRPM;

    if ((desiredRPM > 0 && previousRPM < 0) || (desiredRPM < 0 && previousRPM > 0)) { // if robot trying to sudden change direction
        return 0;
    }
    if (diff > maxAccel){   // if the difference is greater than the max acceleration
        if(power == 0) {
            return desiredRPM; // let robot do its thing b/c it wont take power
        }
        return previousRPM + maxAccel;
    }
    else if (diff < -maxAccel) {
        if(power == 0) {
            return desiredRPM; // let robot do its thing b/c it wont take power
        }
        return previousRPM - maxAccel;
    }
    else {
        return desiredRPM; // under acceleration cap
    }
}
```

## P_Theory Function
```c++
float ChassisSubsystem::p_theory(int LeftFrontPower, int RightFrontPower, int LeftBackPower, int RightBackPower, int LeftFrontRpm, int RightFrontRpm, int LeftBackRpm, int RightBackRpm){
    float krpm2 = 0.000000000616869908524917;
    float kpwr2 = 2.8873053310419543e-26;
    float kboth = 0.00000000679867734389254;
    float a = 0.019247609510979;


    float p1 =  (kboth * LeftFrontPower * LeftFrontRpm) +  (krpm2 * LeftFrontRpm * LeftFrontRpm) +  (kpwr2 * LeftFrontPower * LeftFrontPower) + a;
    float p2 =  (kboth * RightFrontPower * RightFrontRpm) +  (krpm2 * RightFrontRpm * RightFrontRpm) +  (kpwr2 * RightFrontPower * RightFrontPower) + a;
    float p3 =  (kboth * LeftBackPower * LeftBackRpm) +  (krpm2 * LeftBackRpm * LeftBackRpm) +  (kpwr2 * LeftBackPower * LeftBackPower) + a;
    float p4 =  (kboth * RightBackPower * RightBackRpm) +  (krpm2 * RightBackRpm * RightBackRpm) +  (kpwr2 * RightBackPower * RightBackPower) + a;
    
    float p_tot = p1 + p2 + p3 + p4; 

    float A = 224.9;
    float B = 215.8;
    float C = 0.7955;

    float p_tot_c = (A * p_tot * p_tot) + (B * p_tot) + C;

    return p_tot_c;
}

```

## Bisection function 
```c++
float ChassisSubsystem::Bisection(int LeftFrontPower, int RightFrontPower, int LeftBackPower, int RightBackPower, int LeftFrontRpm, int RightFrontRpm, int LeftBackRpm, int RightBackRpm, float chassisPowerLimit) {
    float scale = 0.5; // initial scale
    float precision = 0.25; // initial precision
    float powerInit = p_theory(LeftFrontPower, RightFrontPower, LeftBackPower, RightBackPower, LeftFrontRpm, RightFrontRpm, LeftBackRpm, RightBackRpm);


    if (powerInit > chassisPowerLimit) {

        float powerScaled = p_theory(LeftFrontPower*scale, RightFrontPower*scale, LeftBackPower*scale, RightBackPower*scale, LeftFrontRpm, RightFrontRpm, LeftBackRpm, RightBackRpm);
        
        for (int i = 0; i < 6; i++) {
            
            // if (abs(powerScaled - chassisPowerLimit) < 0.1) {
            //     printf("ppppp \n");
            //     break;
            // }
            if (powerScaled > chassisPowerLimit) {
                scale = scale - precision;
                precision = precision / 2;
                powerScaled = p_theory(LeftFrontPower*scale, RightFrontPower*scale, LeftBackPower*scale, RightBackPower*scale, LeftFrontRpm, RightFrontRpm, LeftBackRpm, RightBackRpm);
                // printf("powerScaled Down: %f\n", powerScaled);
            }

            else { // power is low enough
                scale = scale + precision;
                precision = precision / 2;
                powerScaled = p_theory(LeftFrontPower*scale, RightFrontPower*scale, LeftBackPower*scale, RightBackPower*scale, LeftFrontRpm, RightFrontRpm, LeftBackRpm, RightBackRpm);
                // printf("powerScaled Up: %f\n", powerScaled);
            }
        }
        return scale;
    }

    else {
        return 1;
    }

}
```

## Set Wheel Speeds
```c++
float ChassisSubsystem::setWheelSpeeds(WheelSpeeds wheelSpeeds)
{
    desiredWheelSpeeds = wheelSpeeds; // WheelSpeeds in RPM
    int powers[4] = {0,0,0,0};
    uint32_t time = us_ticker_read();


    powers[0] = motorPIDtoPower(LEFT_FRONT,wheelSpeeds.LF, (time - lastPIDTime));
    powers[1] = motorPIDtoPower(RIGHT_FRONT,wheelSpeeds.RF, (time - lastPIDTime));
    powers[2] = motorPIDtoPower(LEFT_BACK,wheelSpeeds.LB, (time - lastPIDTime));
    powers[3] = motorPIDtoPower(RIGHT_BACK,wheelSpeeds.RB, (time - lastPIDTime));
    

    float LFrpm = limitAcceleration(wheelSpeeds.LF, previousRPM[0], powers[0]);
    float RFrpm = limitAcceleration(wheelSpeeds.RF, previousRPM[1], powers[1]);
    float LBrpm = limitAcceleration(wheelSpeeds.LB, previousRPM[2], powers[2]);
    float RBrpm = limitAcceleration(wheelSpeeds.RB, previousRPM[3], powers[3]);

    // float LFrpm = wheelSpeeds.LF;
    // float RFrpm = wheelSpeeds.RF;
    // float LBrpm = wheelSpeeds.LB;
    // float RBrpm = wheelSpeeds.RB;

    
    // float LFrpm = limitAcceleration(wheelSpeeds.LF, previousRPM[0], LF.getData(POWEROUT));
    // float RFrpm = limitAcceleration(wheelSpeeds.RF, previousRPM[1], RF.getData(POWEROUT));
    // float LBrpm = limitAcceleration(wheelSpeeds.LB, previousRPM[2], LB.getData(POWEROUT));
    // float RBrpm = limitAcceleration(wheelSpeeds.RB, previousRPM[3], RB.getData(POWEROUT));

    previousRPM[0] = LFrpm;
    previousRPM[1] = RFrpm;
    previousRPM[2] = LBrpm;
    previousRPM[3] = RBrpm;

    powers[0] = motorPIDtoPower(LEFT_FRONT,LFrpm, (time - lastPIDTime));
    powers[1] = motorPIDtoPower(RIGHT_FRONT,RFrpm, (time - lastPIDTime));
    powers[2] = motorPIDtoPower(LEFT_BACK,LBrpm, (time - lastPIDTime));
    powers[3] = motorPIDtoPower(RIGHT_BACK,RBrpm, (time - lastPIDTime));
    lastPIDTime = time;

    int p1 = abs(powers[0]);
    int p2 = abs(powers[1]);
    int p3 = abs(powers[2]);
    int p4 = abs(powers[3]);

    int r1 = abs(LF.getData(VELOCITY));
    int r2 = abs(RF.getData(VELOCITY));
    int r3 = abs(LB.getData(VELOCITY));
    int r4 = abs(RB.getData(VELOCITY));

    float scale = Bisection(p1, p2, p3, p4, r1, r2, r3, r4, power_limit);

    // printf("Before Set:%.3f\n", p_theory(p1*scale, p2*scale, p3*scale, p4*scale, r1, r2, r3, r4));

    LF.setPower(powers[0]*scale);
    RF.setPower(powers[1]*scale);
    LB.setPower(powers[2]*scale);
    RB.setPower(powers[3]*scale);

    p1 = abs(LF.getData(POWEROUT));
    p2 = abs(RF.getData(POWEROUT));
    p3 = abs(LB.getData(POWEROUT));
    p4 = abs(RB.getData(POWEROUT));
    // printf("After Set:%.3f\n", p_theory(p1, p2, p3, p4, r1, r2, r3, r4));

    return scale;
}
```

