#include "Motors.h"
#include "LineSensors.h"
#include "PID.h"
#include "Encoders.h"
#include "Kinematics.h"
#include "lcd.h"

//Bayesian Filter

#define SENSORS 5
#define SAMPLES 50

LineSensors_c line_Sensors;
Motors_c motors;
PID_c pid;
Kinematics_c pose;
LCD_c display(0,1,14,17,13,30);

float mean[SENSORS] = {};
float diff_sq[SENSORS] = {};
float Z_value[SENSORS];
int count = 0;



float var[SENSORS];
float std_deviation[SENSORS];
float post_left;
float post_center;
float post_right;

float likelihood_left;
float likelihood_center;
float likelihood_right;
float LikeliHood[SENSORS];



unsigned long CALIB_TIME = 10000;


void setup()
{
  Serial.begin(9600);
  pid.initialise(0, 0, 0);
  setupEncoder0();
  setupEncoder1();
  pose.initialise(0,0,0);

  line_Sensors.initialiseForADC();
  for(int i = 0; i < SENSORS; i++)
  {
    mean[i] = 0.0;
    diff_sq[i] = 0.0;
    std_deviation[i] = 1.0;
    var[i] = 1.0;
  }

  

  SelfCalibration();
  for(int i = 0; i < SENSORS; i++)
  {
    if(count > 1 )
    {
      var[i] = (diff_sq[i] / (count - 1)); 
      if(var[i] <= 0.0f)
      {
        var[i] = 1.0f;   
      }
      std_deviation[i] = sqrt(var[i]);
    }
    else
    {
      std_deviation[i] = 1.0;
      var[i] = 1.0f;
    }
    
    
    
    
    Serial.print("Sensor");
    Serial.print(i);
    Serial.print(" - ");
    Serial.print("Mean:");
    Serial.print(mean[i]);
    Serial.print(",");
    Serial.print("std_deviation:");
    Serial.print(std_deviation[i]);
  }
  delay(30);

 


    
}

void loop()
{
  calculateZ();
  
  for(int i = 0; i < SENSORS; i++)
  {
    //Serial.print("Reading "); 
    //Serial.print(i);
    //Serial.println(" : ");
    //Serial.println(line_Sensors.readings[i]);
    //Serial.print(" Mean ");
    //Serial.print(i);
    //Serial.print(":"); 
    //Serial.print(mean[i]);
    //Serial.println("Std_dev");
    //Serial.print(i);
    //Serial.print(":");
    //Serial.println(std_deviation[i]);
    
    
  } 
  likeliHood();
  pose.update();
  Posecalculation();
  
  delay(200);

  
  
  
  
    
}

void SelfCalibration()
{
  unsigned long Start_Time = millis();

  count = 0;
  
 

  while((millis() - Start_Time) < CALIB_TIME )
  {
    line_Sensors.readSensorsADC();
    count++;
    

    for(int j = 0; j < SENSORS; j++)
   {
     float readings = line_Sensors.readings[j];
     float Diff = readings - mean[j];
     mean[j] += Diff / count;

    
     diff_sq[j] += Diff *( readings - mean[j]);

    }
     delay(10);
   }
   //Serial.println("Calibration time completed");
    
  }

void calculateZ()
{
  line_Sensors.readSensorsADC();
  for(int i = 0; i < SENSORS; i++)
  {
    
    float S_dev = std_deviation[i];
    if(S_dev <= 1.0)
    {
      S_dev = 1.0;
      
    }
    
   
    Z_value[i] = (line_Sensors.readings[i] - mean[i]) / S_dev;

    //Serial.println("Calculation of Z scores done!");
    //Serial.print("Z");
    //Serial.print(i);
    //Serial.print(" : ");
    //Serial.print(Z_value[i]);
    
    
    
    
  }
  
}

void likeliHood()
{
  
  for(int i = 0; i < SENSORS; i++)
  {
    
    LikeliHood[i] = (-0.5f * Z_value[i] * Z_value[i]); 
  }
  likelihood_left = LikeliHood[0];
  likelihood_center = LikeliHood[2];
  likelihood_right =  LikeliHood[4];
  //Serial.println("Likelihood is determined");

  float prior = logf(0.3);
  float P_1 = likelihood_left + prior;
  float P_2 = likelihood_center + prior;
  float P_3 = likelihood_right + prior;
  P_1 = abs(P_1);
  P_2 = abs(P_2);
  P_3 = abs(P_3);
  //Serial.println("Prior X Evidence:");
  //Serial.print("P_1");
  //Serial.print("=");
  //Serial.print(P_1);
  //Serial.print("P_2");
  //Serial.print("=");
  //Serial.print(P_2);
  //Serial.print("P_3");
  //Serial.print("=");
  //Serial.print(P_3);
  
  float maximumP = fmax(P_1,fmax(P_2,P_3));
  
  
  
  float normalize_Left = expf(P_1 - maximumP);
  //Serial.println("norm_left");
  //Serial.print(norm_Left);
  float normalize_Center = expf(P_2 - maximumP);
  //Serial.println(normalize_Center);
  float normalize_Right = expf(P_3 - maximumP);

  float normalize_P = normalize_Left + normalize_Center + normalize_Right;
  //Serial.println("norm_p");
  //Serial.print(norm_P);
  post_left = normalize_Left / normalize_P;
  //Serial.println("post_left:");
  //Serial.print(post_left);
  post_center = normalize_Center / normalize_P;
  post_right = normalize_Right / normalize_P;
  //Serial.print("Posterior Probability of left :");
  //Serial.println(post_left);
  //Serial.print("Posterior Probability of Center :");
  //Serial.println(post_center);
  //Serial.print("Posterior Probability of right :");
  //Serial.println(post_right);
  
  

  float Threshold = 0.6;
  float maxPosterior = fmaxf(post_left, fmaxf(post_center, post_right));

  if(Threshold > maxPosterior)
  {
    //Serial.println("Leader not detected");
    motors.setPWM(0.0, 0.0); 
    
  }
  else
  {
    if(post_left > Threshold && post_left > post_center && post_left > post_right)
    {
      //Serial.println("Leader is detected in the left");
      motors.setPWM(-20.0, 20.0);
      
    }
    else if(post_center > Threshold && post_center > post_left && post_center > post_right)
    {
      //Serial.println("Leader is detected in the center");
      motors.setPWM(20.0, 20.0);
      
    }
    else if(post_right > Threshold && post_right > post_center && post_right > post_left)
    {
      //Serial.println("Leader is detected in the right");
      motors.setPWM(20.0,-20.0);
      
    }
    
  }   
  
}
void Posecalculation()
{
  //display.gotoXY(0, 0);
  //display.print("X:");
  //display.print(pose.x);
  //display.gotoXY(0, 1);
  //display.print("Y:");
  //display.print(pose.y);
  //display.gotoXY(0,2);
  //display.print("Theta:");
  //display.print(pose.theta); 
  Serial.println("X:");
  Serial.print(pose.x);
  Serial.println("Y:");
  Serial.print(pose.y);
  Serial.println("Theta:");
  Serial.print(pose.theta); 
  
  
}

  
  
  
  
  
