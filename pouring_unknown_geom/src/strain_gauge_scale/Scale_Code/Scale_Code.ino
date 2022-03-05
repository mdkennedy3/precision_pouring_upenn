#include <ros.h>
#include "HX711.h"
#include "RunningAverage.h"
#include "RunningMedian.h"
#include <pouring_unknown_geom/ScaleMsg.h>

//For Ros:
ros::NodeHandle nh;
pouring_unknown_geom::ScaleMsg scale_msg;
ros::Publisher scale_pub("arduino_scale", &scale_msg);


RunningAverage myRA(3);
//RunningMedian samples = RunningMedian(3);

int sample = 0;

HX711 Scale;

void setup() {
  Serial.begin(38400);
  Serial.println("HX711 Demo");
  myRA.clear();
  // Initialize Pins
  Scale.begin(A1, A0);

  // Tare System
//  Serial.println("Get ready to zero system in ...");
//  delay(1500);
//  Serial.println("3...");
//  delay(1500);
//  Serial.println("2...");
//  delay(1500);
//  Serial.println("1...");
//  delay(1500);
//  Serial.println("Zeroing System");
//  
  Scale.set_scale(400.f);      // 410 // this value is obtained by calibrating the scale with known weights; see the README for details
  Scale.tare();				        // reset the scale to 0

  Serial.println("System Zeroed");
  Serial.println("Start Readings: (in grams)");

  //For ROS:
  nh.initNode();
  nh.advertise(scale_pub);
}

// Define variables
float weight;

// float threshold = 3;
void loop() {
    
// X axis / Right Sensor Calculations
  weight = Scale.get_units(1);     // Get Raw Input Average for X axis / Right Sensor
  weight = -weight;

//  samples.add(weight);
//  Serial.print("Running Median: ");
//  Serial.println(samples.getMedian());  

  myRA.addValue(weight);
//  samples++
//
  Serial.print("Running Average: ");
  Serial.println(myRA.getAverage(),3);

  //For ROS: 
  scale_msg.header.stamp = nh.now();
  scale_msg.header.frame_id = "arduino_scale";

  //scale_msg.mass = 9.81*weight/1000.0; //Convert back to kg for Newtons
  scale_msg.mass = 0.181184336*myRA.getAverage() +1.02917285 ; // weight; //Convert back to kg for Newtons
  
  scale_msg.units = "g";


  scale_pub.publish(&scale_msg);
  nh.spinOnce();





//long runningAverage(int weight) {
//  #define LM_SIZE 3
//  static int LM[LM_SIZE];
//  static byte index = 0;
//  static long sum = 0;
//  static byte count = 0;
//
//  // keep sum updated to improve speed
//  sum -= LM[index];
//  LM[index] = weight;
//  sum += LM[index];
//  index++;
//  index = index % LM_SIZE;
//  if (count < LM_SIZE) count++;
//
//  return sum / count;
//}



// Print Results of all Sensors
  //Serial.print("Scale Reading:\t");
//  Serial.print(weight, 1);
//  Serial.println(" grams");

}
