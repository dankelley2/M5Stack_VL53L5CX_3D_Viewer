/*
  Read an 8x8 array of distances from the VL53L5CX
  Output as float[3] array with normalized x,y coordinates + z distance
  Display
*/
#include <M5Stack.h>
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX

#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef max
#define max(a, b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef SENSOR_SIZE
#define SENSOR_SIZE 64
#endif

#ifndef SENSOR_FOV
#define SENSOR_FOV 45
#endif

enum Mode { scan, view, off };
Mode currentMode = off;

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM

int imageResolution = 0; // Used to pretty print output
int imageWidth = 0;      // Used to pretty print output

int points_raw[SENSOR_SIZE];
int points_screen[SENSOR_SIZE][3]; // Includes z depth for color
int z_buffer[SENSOR_SIZE]; // Includes z depth for color
float points_3d[SENSOR_SIZE][3];
float angle_deg_x = 0;           // rotation around the X axis
float angle_deg_y = 0;           // rotation around the Y axis
float angle_deg_z = 90;           // rotation around the Z axis
int dir = 1;
float z_offset = 0.0;            // offset on Z axis
float cube_size = 150.0;           // cube size (multiplier)
float meanDistance = 0.0;
float tmpMeanDistance = 0.0;
int maxDistance = 800;
int maxZValue = 0;
float TZValue = 0;

float LERP_FACTOR = 0.5;

int threshold = 0;

bool haveData = false;

void setup()
{
  //Init M5 with Screen on, speaker off, i2c on, serial on
  M5.begin(true, false, true, true);

  //Delay Startup
  delay(500);

  //Init wire
  Wire.begin(21, 22); // This resets I2C bus to 100kHz

  //Set i2c clock
  Wire.setClock(1000000); //Sensor has max I2C freq of 1MHz

  Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
  if (myImager.begin() == false)
  {
    Serial.println(F("Sensor not found - check your wiring. Freezing"));
    while (1)
      ;
  }

  myImager.setResolution(SENSOR_SIZE); // 4*4 or 8*8

  imageResolution = myImager.getResolution(); // Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution);         // Calculate printing width

  // Using 4x4, min frequency is 1Hz and max is 60Hz
  // Using 8x8, min frequency is 1Hz and max is 15Hz
  myImager.setRangingFrequency(15);

  //m5 screen Setup
  M5.Lcd.fillScreen(BLACK);

}

//Draw 3d points on the screen
void draw3dPoints() {

  //Draw Points
  for (int i = 0; i < imageResolution; i++) {
        if (points_screen[i][2] <= 0) continue;
        
        int rs = map(points_screen[i][2],0,maxDistance,5,1);
        uint16_t shade;

        //if we have a point less than mean
        if (meanDistance != 0 && points_screen[i][2] < meanDistance){
          shade = GREEN;
        }
        else{
          shade = shadeToRGB(map(points_screen[i][2],0,maxDistance,254,10));
        }
        M5.Lcd.fillCircle(points_screen[i][0], points_screen[i][1], rs, shade);
  }
}

//Draw 3d points on the screen
void drawMesh(uint16_t color) {
  //Draw Points
  int lowerPixel = imageWidth;
  int diagPixel = imageWidth + 1;
  int nextPixel = 1;
  
  for (int i = 0; i < imageResolution; i++) {
    if (points_screen[i][2] <= 0) continue;

    //Draw to lower stride
    if (i < imageResolution - imageWidth){
      
      M5.Lcd.drawLine(points_screen[i][0], points_screen[i][1], points_screen[lowerPixel][0], points_screen[lowerPixel][1], color);
      
      //if on last Pixel, don't attempt to draw lines to next or diag
      if (i % imageWidth != imageWidth - 1){
        //Draw to Next adjacent
        M5.Lcd.drawLine(points_screen[i][0], points_screen[i][1], points_screen[nextPixel][0], points_screen[nextPixel][1], color);
        //Draw to lower stride next adjacent
        M5.Lcd.drawLine(points_screen[i][0], points_screen[i][1], points_screen[diagPixel][0], points_screen[diagPixel][1], color);
      } 
      //On last row now
    } else if (i % imageWidth != imageWidth - 1){
      //Just draw to next
      M5.Lcd.drawLine(points_screen[i][0], points_screen[i][1], points_screen[nextPixel][0], points_screen[nextPixel][1], color);
    }
    lowerPixel++;
    diagPixel++;
    nextPixel++;
  }
}

void drawStats(){
  //Draw Stats
  M5.Lcd.setCursor(1,3);
  M5.Lcd.printf("True Max Z Value = %f",TZValue);
  M5.Lcd.setCursor(1,12);
  M5.Lcd.printf("Z Offset         = %f",z_offset);
}

//Draw 3d points on the screen
void resetScreenPoints() {
  for (int i = 0; i < imageResolution; i++) {
      if (points_screen[i][2] <= 0) continue;
      int rs = map(points_screen[i][2],0,maxDistance,5,1);
      M5.Lcd.fillCircle(points_screen[i][0], points_screen[i][1], rs, BLACK);
  }
}

void loop()
{

  //Try Get Data
  if (currentMode == scan) {
    DisplaySensorData();
  }
  else if (currentMode == view) {
    //Draw if exists
    if (haveData) {
      DrawData();
    }
  }

  // increase the angle slowly
//    angle_deg_y += 0.1f *  dir;
//    if (angle_deg_y >= 30 || angle_deg_y <= -30) {
//      dir *= -1;
//    }


  //Let M5 update.
  M5.update();
  PollButtons();
}

void DrawData() {
  //Reset the points on the screen by drawing over them
  drawMesh(BLACK);
  //resetScreenPoints();

  //Transform point array to 3d
  calculate3dPoints();

  //Draw Mesh first
  drawMesh(WHITE);

  //Draw on screen
  //draw3dPoints();

  //Draw stats
  drawStats();
  delay(5); // Small delay between polling
}

void DisplaySensorData() {
  // Poll sensor for new data
  if (myImager.isDataReady() == true)
  {
    if (myImager.getRangingData(&measurementData)) // Read distance data into array
    {
      for (int i = 0; i < SENSOR_SIZE; i++)
      {
          //Get Real Coordinates
          // Image will be flipped horizontally and vertically in this scenario
          int x = i % imageWidth;
          int y = i / imageWidth;
          
          //Get mm
          int mm = measurementData.distance_mm[i];

          //Add points to array, lerp to value by factor
          int c_mm = constrain(mm, 0, maxDistance);
          points_raw[i] = round(lerp((float)points_raw[i], (float)c_mm, LERP_FACTOR));
      }
      
      haveData = true;
      DrawData();
    }
  }
}

float* rotateX(float& y, float& z, float angle)
{
  float s = sin(radians(angle));
  float c = cos(radians(angle));
  float* result = new float[2];
  result[0] = y * c - y * s;
  result[1] = y * s + z * c;
  return result;
}

float* rotateY(float& x, float& z, float angle)
{
  float s = sin(radians(angle));
  float c = cos(radians(angle));
  float* result = new float[2];
  result[0] = x * c + z * s;
  result[1] = z * c - x * s;
  return result;
}

float* rotateZ(float& x, float& y, float angle)
{
  float s = sin(radians(angle));
  float c = cos(radians(angle));
  float* result = new float[2];
  result[0] = x * c - y * s;
  result[1] = x * s + y * c;
  return result;
}

void calculate3dPoints() {
  
  float startAngle = -((float)SENSOR_FOV) / 2;
  float angleInc = ((float)SENSOR_FOV) / (imageWidth - 1);
  maxZValue = 0;
  TZValue = 0;
  float sum = 0.0;
  
  for (int i = 0; i < imageResolution; i++) {

    //Back into coords
    int ix = (i % imageWidth);
    int iy = (i / imageWidth);
    
    float rx = deg2rad(startAngle + ix * angleInc);
    float ry = deg2rad(startAngle + iy * angleInc);

    float dist = points_raw[i];
    
    points_3d[i][0] = dist * sin(rx) * cos(ry);
    points_3d[i][1] = dist * sin(ry);
    points_3d[i][2] = dist * cos(rx) * cos(ry);
    maxZValue = max(maxZValue,(int)points_3d[i][2]);

    points_3d[i][2] = points_3d[i][2] + z_offset;
    
    //declare points for transformation
    float x1, y1, z1;
    x1 = points_3d[i][0];
    y1 = points_3d[i][1];
    z1 = points_3d[i][2];

    //Rotate on Z
    float* temp_Rotation = rotateZ(x1, y1, angle_deg_z);
    x1 = temp_Rotation[0];
    y1 = temp_Rotation[1];
    delete temp_Rotation;
    
    //Rotate on Y
    temp_Rotation = rotateY(x1, z1, angle_deg_y);
    x1 = temp_Rotation[0];
    z1 = temp_Rotation[1];
    delete temp_Rotation;
    
    //Rotate on X
    temp_Rotation = rotateX(y1, z1, angle_deg_x);
    y1 = temp_Rotation[0];
    z1 = temp_Rotation[1];
    delete temp_Rotation;
    
    //reassign
    points_3d [i][0] = x1;
    points_3d [i][1] = y1;
    points_3d [i][2] = z1;

    TZValue = max(TZValue,z1);
    
    // project 3d points into 2d space with perspective divide -- 2D x = x/z,   2D y = y/z
//    points_screen[i][0] = round(M5.Lcd.width() / 2 + points_3d [i][0] * cube_size / points_3d [i][2]);
//    points_screen[i][1] = round(M5.Lcd.height() / 2 + points_3d [i][1] * cube_size / points_3d [i][2]);
//    points_screen[i][2] = constrain(map((int)(points_3d [i][2] * 100), 0, maxDistance, 255, 0), 10, 254);
//    
    points_screen[i][0] = round(M5.Lcd.width() / 2 + points_3d [i][0] * cube_size / points_3d [i][2]);
    points_screen[i][1] = round(M5.Lcd.height() / 2 + points_3d [i][1] * cube_size / points_3d [i][2]);
    points_screen[i][2] = round(points_3d[i][2]); 
    
    sum += points_3d[i][2];
  }
  tmpMeanDistance = sum / SENSOR_SIZE;
}

void PollButtons() {
  if (M5.BtnA.wasReleased()) {
    if (currentMode != scan){
      wakeImager();
      //angle_deg = 0;
      z_offset = -30.0;
      currentMode = scan;
    } else {
      setMean();
    }
  }
  //if long press, set threshold based on center average
//  if (M5.BtnA.pressedFor(1000, 500)){
//    threshold = mainCenterAverage;
//    Serial.println("Threshold Set");
//  }
  if (M5.BtnB.wasReleased() || M5.BtnB.pressedFor(500, 10)) {
    //    sleepImager();
    //    currentMode = view;
    zoomIn();
  }
  if (M5.BtnC.wasReleased() || M5.BtnC.pressedFor(500, 10)) {
    zoomOut();
  }
}

void zoomIn() {
  z_offset -= 0.1;
}

void zoomOut() {
  z_offset += 0.1;
}

void wakeImager() {
  myImager.setPowerMode(SF_VL53L5CX_POWER_MODE::WAKEUP);
  myImager.startRanging();
}

void sleepImager() {
  myImager.stopRanging();
  myImager.setPowerMode(SF_VL53L5CX_POWER_MODE::SLEEP);
}

void setMean() {
  meanDistance = tmpMeanDistance;
}

//0-255 value to a greyscale uint_16_t color shade
uint16_t shadeToRGB(int shade) {
  uint16_t red_565 = map(shade, 0, 255, 0, 31);
  uint16_t green_565 = map(shade, 0, 255, 0, 63);
  uint16_t blue_565 = map(shade, 0, 255, 0, 31);

  return (uint16_t)(red_565 << 11) | (green_565 << 5) | blue_565;
}

float deg2rad(float d)
{
    float pi = 3.14159265359;
    return (d * (pi / 180.0));
}

float lerp(float a, float b, float f)
{
    return a + f * (b - a);
}
