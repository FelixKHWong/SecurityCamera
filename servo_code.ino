#include <Servo.h>
#include <Wire.h>

Servo servo_x;
Servo servo_y;

//need to find address of arduino(pin used)
#define SLAVE_ADDRESS 0x08

int servo_x_pin = 9;
int servo_y_pin = 10;

int servo_x_position = 0;
int servo_y_position = 0;

int capture_pin = A0;
byte capture_flag = 0;

//set default value to 512 which should be middle of joystick
int data_from_pi = 512;

void setup() {
  //setting up logging
  Serial.begin(9600);
  servo_x.attach(servo_x_pin);
  servo_y.attach(servo_y_pin);

  Wire.begin(SLAVE_ADDRESS);
  int new_x_pos = 90;
  int new_y_pos = 90;
  Serial.print("Completed Inititialization");

  Wire.onReceive(receive_data);
  Wire.onRequest(send_data);

}


void write_to_servo(Servo servo, int value)
{
  servo.write(value);
  Serial.print(servo.read());
}


void receive_data(int num_bytes)
{
  while (Wire.available())
  {
    data_from_pi += Wire.read();
  }
  //find way to split byte data? Use array to hold x and y data?
  int new_x_pos = 0;
  int new_y_pos = 0;
  
  //write x
  write_to_servo(servo_x, new_x_pos);

  //write to y
  write_to_servo(servo_y, new_y_pos);

}

//read button press and send data to pi
void send_data()
{
  capture_flag = analogRead(capture_pin);
  if (capture_flag)
  {
    Wire.write(capture_flag);
  }
  capture_flag = 0;
}


void loop() {
  delay(300);
  //not sure if I can add some more processing here, processing will probably go in send_data function
}
