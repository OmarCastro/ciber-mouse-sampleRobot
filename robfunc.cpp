#include "robfunc.h"
bool waitingForSensor = false;
/* Calculate the power of left and right motors */
void DetermineAction(int beaconToFollow, float *lPow, float *rPow, int *state)
{
    static int counter=0;
    static float InitalOrientation = 0.0;

    bool   beaconReady;
    static struct beaconMeasure beacon; // beacon sensor
    static float  left; //value of frontal left sonar sensor
    static float right; //value of frontal rigth sonar sensor
    static float center; //value of frontal center sonar sensor
    static int    Ground;
    static bool   Collision;// collision sensor
    static float Compass; //compass sensor


    

  // SENSORS ACCESS

    /*Access to values from Sensors - Only ReadSensors() gets new values */
    if(IsObstacleReady(LEFT))
        left=     GetObstacleSensor(LEFT);
    if(IsObstacleReady(RIGHT))
        right=    GetObstacleSensor(RIGHT);
    if(IsObstacleReady(CENTER))
        center=   GetObstacleSensor(CENTER);

    beaconReady = IsBeaconReady(beaconToFollow);
    if(beaconReady) {
       beacon =  GetBeaconSensor(beaconToFollow);
    }
    else beaconReady=0;

    if(IsGroundReady())
        Ground=    GetGroundSensor();
    if(IsBumperReady())
        Collision= GetBumperSensor();
    if(IsCompassReady()){
        Compass= GetCompassSensor();
        if(InitalOrientation == 0.0){
            /* its very rare for the initial orientation to be exactly 0.0
             * an aproximated value of orientation is enough
            */
            InitalOrientation = Compass;
        }
    }











    else if(center>4.0 || right> 3.1 || left>3.1 || Collision) { /* Close Obstacle - Rotate */
        if(InitalOrientation == 0.0){
            waitingForSensor == true;
        }


        if(center < 0.7){
            if(*state == BYPASSIN_RIGTH){
                if(left < 3.7 ){
                    *lPow=0.04;
                    *rPow=0.06;
                } else if(left > 4.5 ){
                    *lPow=0.06;
                    *rPow=0.04;
                } else {
                    *lPow=0.05;
                    *rPow=0.05;
                }
            } else {
                if(right < 3.7 ){
                    *lPow=0.06;
                    *rPow=0.04;
                } else if(right > 4.5 ){
                    *lPow=0.04;
                    *rPow=0.06;
                } else {
                    *lPow=0.05;
                    *rPow=0.05;
                }
            }
        } else if(right < left) {
               *lPow=0.06;
               *rPow=-0.06;
               *state = BYPASSIN_RIGTH;
        }  else {
               *lPow=-0.06;
               *rPow=0.06;
               *state = BYPASSIN_LEFT;
        }
    }

    else if(center > 1.0){
        if(right > left){
            *lPow=0.02;
            *rPow=0.05;
        } else {
            *lPow=0.05;
            *rPow=0.02;
        }
    }

    else if(beaconReady && beacon.beaconVisible){
        if(beacon.beaconDir>20.0){
            *lPow=0.0;
            *rPow=0.1;
        }
        else if(beacon.beaconDir<-20.0){
            *lPow=0.1;
            *rPow=0.0;
        }
        else { /* Full Speed Ahead */
           *lPow=0.15;
           *rPow=0.15;
        }
    } else {
        if(*state != RUNNING){
            float diff = Compass - InitalOrientation; //difference between initial orientation and current
            if(diff < 10 && diff > -10){
                *state = RUNNING;
            }
        }

        if(*state == BYPASSIN_LEFT){
            *lPow=0.07;
            *rPow=0.01;

        }
        else if(*state == BYPASSIN_RIGTH){
            *lPow=0.01;
            *rPow=0.07;
        }
        else { /* Full Speed Ahead */
           *lPow=0.1;
           *rPow=0.1;
        }
    }




    counter++;
}
