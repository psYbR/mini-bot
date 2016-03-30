#include "Wire.h" //I2C
#include "Adafruit_Sensor.h" //for accelerometer
#include "Adafruit_MMA8451.h" //accelerometer
#include "Romeo_m.h" //motor controller
#include "Adafruit_NeoPixel.h" //neopixel
#include "LiquidCrystal_I2C.h"

#define ir_left 1 //analog pin
#define ir_right 2 //analog pin
#define neopixel_pin 13 //digital pin
#define neopixel_num 12 //number of neopixels to address
#define piezo_pin 12 //digital pin

#define duty_cycle 150 //motor duty cycle
#define duty_cycle_max 180 //the max duty cycle for when going forward straight
#define adj_reverse 0 //additional duty cycle for the reversing side in a turn

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(neopixel_num, neopixel_pin, NEO_GRB + NEO_KHZ800); //neopixels
Adafruit_MMA8451 mma = Adafruit_MMA8451(); //accelerometer
sensors_event_t event; //set up a sensor event object for the accelerometer
LiquidCrystal_I2C  lcd(0x3f,2,1,0,4,5,6,7); //0x27 is the default I2C bus address for an unmodified backpack, others use 0x3f. The other arguments are the pin mapping for the I2C backpack

//notes for startup sound
int tones[] = {392, 523, 659, 784};

void setup() { 

  //init stuff
  Romeo_m.Initialise();
  pixels.begin(); // This initializes the NeoPixel library.
  mma.begin(); //accelerometer
  mma.setRange(MMA8451_RANGE_8_G);
  lcd.begin (20,4); //for 20 x 4 LCD module
  lcd.setBacklightPin(3, POSITIVE); //for the regular LCDs
  lcd.setBacklight(HIGH); //turn the backlight on
  lcd.print("I am currently HAPPY");
  
  show_smiley(100); //put a smile on the minibot's dial

  pinMode (ir_left, INPUT);
  pinMode (ir_right, INPUT);
  Serial.begin(115200); //pour a bowl of Serial

  for (int i = 0; i < 4; i++) {
    tone(piezo_pin, tones[i]);
    delay(100);
  }
  delay(150);
  noTone(piezo_pin);
  
}

//shows a smiley on the neopixel ring
void show_smiley (int brightness) {
  pixels.setPixelColor(0, pixels.Color(brightness,0,brightness)); //set pixel colour / brightness
  pixels.setPixelColor(2, pixels.Color(brightness,0,brightness)); //set pixel colour / brightness
  pixels.setPixelColor(5, pixels.Color(brightness,0,brightness)); //set pixel colour / brightness
  pixels.setPixelColor(7, pixels.Color(brightness,0,brightness)); //set pixel colour / brightness
  pixels.setPixelColor(8, pixels.Color(brightness,0,brightness)); //set pixel colour / brightness
  pixels.setPixelColor(9, pixels.Color(brightness,0,brightness)); //set pixel colour / brightness
  pixels.setPixelColor(10, pixels.Color(brightness,0,brightness)); //set pixel colour / brightness
  pixels.setPixelColor(11, pixels.Color(brightness,0,brightness)); //set pixel colour / brightness
  pixels.show(); //send the updated pixel color to the hardware.
}

//store values for the IR sensors
int current_avg_value = 0; //which key in the average array to use
int vals_left[] = {0,0,0,0,0,0}; //stores range values from the IR sensor so we can calculate an average of the last six values
int vals_right[] = {0,0,0,0,0,0};

//updates the average values of the IR sensor range
void range_update () {
  vals_left[current_avg_value] = get_range("left"); //get value from the interrupt
  vals_right[current_avg_value] = get_range("right"); //get value from the interrupt
  if (current_avg_value == 5) {
    current_avg_value = 0; //reset the value to update
  }
  else{
    current_avg_value++; //increment the value we are saving into
  }
}

//returns the average range for an IR sensor
int get_avg_range ( String side ) {

  if (side == "left") { 
    return (int)((vals_left[0] + vals_left[1] + vals_left[2] + vals_left[3] + vals_left[4] + vals_left[5]) / 6.00);
  }
  else if (side == "right") {
    return (int)((vals_right[0] + vals_right[1] + vals_right[2] + vals_right[3] + vals_right[4] + vals_right[5]) / 6.00);  
  }
  else { return 0; }
  
}

//get the ranges from the left or right IR sensor
int get_range ( String side ) {
  
  int side_pin;
  if (side == "left") { side_pin = ir_left; }
  else if (side == "right") { side_pin = ir_right; }
  else { return 0; }
  
  int raw = analogRead(side_pin);
  int volt = map(raw, 0, 1023, 0, 5000);
  int range = (21.61/(volt-0.1696))*1000;
  if (range > 30) {
    range = 30;
  }
  return range;
  
}

boolean driving = false; //if the robot should actively move
int long wall_check = 0; //time since last successfully avoided a wall
int wall_negotiate = 5000; //max time (ms) to spend negotiating a wall before doing a 180
int break_range = 15; //IR distance to start avoiding object
String going = "forward";
//int long going_forwards = 0;

void avoid_right () {
  if (going == "forward" || going == "right") {
    Romeo_m.motorControl(Forward,duty_cycle,Reverse,duty_cycle + adj_reverse); //turn right
    going = "right";
  }
  else {
    Romeo_m.motorControl(Reverse,duty_cycle + adj_reverse,Forward,duty_cycle); //turn left
  }
  delay(15);
}

void avoid_left () {
  if (going == "forward" || going == "left") {
    Romeo_m.motorControl(Reverse,duty_cycle + adj_reverse,Forward,duty_cycle); //turn left
    going = "left";
  }
  else {
    Romeo_m.motorControl(Forward,duty_cycle,Reverse,duty_cycle + adj_reverse); //turn right
  }
  delay(15);
}

void loop() {

  range_update();
  
  char val;
  if(Serial.available()>0)
  {  
    val = Serial.read();
  }
  switch(val) {
    case 'a':
      driving = !driving; //toggle the robot moving
    break;
    default: break;
  }

  int left_range = get_avg_range("left");
  int right_range = get_avg_range("right");

  if (driving) {
    if (left_range < break_range && right_range < break_range) {
      Serial.println("Facing wall");
      if (left_range < right_range) {
        avoid_left();
      }
      else {
        avoid_right();
      }      
    }
    else if (left_range < break_range && right_range >= break_range) { // wall on the left
      Serial.println("Wall on left");
      avoid_right();
    }
    else if (left_range >= break_range && right_range < break_range) { // wall on the right
      Serial.println("Wall on right");
      avoid_left();
    }
    else { //no obstacles
      Serial.println("Forwards!");
      Romeo_m.motorControl(Forward,duty_cycle,Forward,duty_cycle); //onward soldier!
      going = "forward";
      wall_check = millis();
    }
    if ((millis() - wall_check) > wall_negotiate) {
      Serial.println("Stuck on wall");
      Romeo_m.motorControl(Reverse,duty_cycle,Reverse,duty_cycle); //onward soldier!
      for (int i = 0; i < 4; i++){
        tone(piezo_pin, 659);
        delay(100);
        tone(piezo_pin, 784);
        delay(100);
      }
      noTone(piezo_pin);
      avoid_left();
      delay(2800); //then turn left
      going = "forward";
      wall_check = millis();
    }
  }
  else {
    Romeo_m.motorStop();
  }

  lcd.setCursor(0,1);
  lcd.print("L: " + (String)left_range + "    ");
  lcd.setCursor(6,1);
  lcd.print("R: " + (String)right_range + "    ");
  mma.getEvent(&event);
  lcd.setCursor(0,2); //go to start of 2nd line
  lcd.print("P: " + (String)event.acceleration.y + "    ");
  lcd.setCursor(10,2); //go to start of 2nd line
  lcd.print("R: " + (String)event.acceleration.x + "    ");
  
  //Serial.println("Range left: " + (String)left_range + ", range right: " + (String)right_range + ".");
  
}



/* writes to neopixels
for(int i=0;i < neopixel_num;i++){ //for each pixel
  delay(10);
  pixels.setPixelColor(i, pixels.Color(brightness,brightness-10,brightness)); //set pixel colour / brightness
  pixels.show(); //send the updated pixel color to the hardware.
}
*/


