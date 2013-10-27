#include "robfunc.h"
float comppass = 0;
bool waitingForSensor = false;
/* Calculate the power of left and right motors */
void DetermineAction(int beaconToFollow, float *lPow, float *rPow, int *state)
{
    static int counter=0;
    static int rotTime=0;
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
    }











    else if(center>4.0 || right>4.1 || left>4.1 || Collision) { /* Close Obstacle - Rotate */
        if(comppass == 0.0){
            waitingForSensor == true;
        }


        if(center < 0.8){
            *lPow=0.05;
            *rPow=0.05;
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

    else if(right>1.5) { /* Obstacle Near - Avoid */
        *lPow=0.00;
        *rPow=0.05;
    }
    else if(left>1.5) {
        *lPow=0.05;
        *rPow=0.00;
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
        if(rotTime >= 5){
            *state = RUNNING;
            rotTime = 0;
        }
        if(*state == BYPASSIN_LEFT){
            *lPow=0.07;
            *rPow=0.00;
            rotTime++;

        }
        else if(*state == BYPASSIN_RIGTH){
            *lPow=0.00;
            *rPow=0.07;
            rotTime++;
        }
        else { /* Full Speed Ahead */
           *lPow=0.1;
           *rPow=0.1;
        }
    }




    counter++;
}
