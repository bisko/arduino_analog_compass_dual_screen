#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <HMC5883L.h>
#include <ADXL345.h>
#include <U8g2lib.h>

Adafruit_BMP085 bmp;
HMC5883L compass;
ADXL345 accelerometer;

/**
 * Screen 1 is used to draw the compass graphic and heading
 */
U8G2_SSD1306_128X64_NONAME_1_HW_I2C screen1( U8G2_R0, U8X8_PIN_NONE );

/**
 * Screen 2 is used to draw text information
 */
U8X8_SSD1306_128X64_NONAME_HW_I2C screen2( U8G2_R0, U8X8_PIN_NONE );

#define compass_centre_x 32
#define compass_centre_y 32
#define compass_radius 15

#define compass_pointer_height 4
#define compass_pointer_width 8

#define compass_calibration_x 97
#define compass_calibration_y -193

void setup() {
  Wire.begin();
  bmp.begin();

  if ( ! accelerometer.begin() ) { delay( 200 ); }
  accelerometer.setRange( ADXL345_RANGE_2G );

  while ( ! compass.begin() ) { delay( 200 ); }
  
  compass.setRange(             HMC5883L_RANGE_1_3GA    );
  compass.setMeasurementMode(   HMC5883L_CONTINOUS      );
  compass.setDataRate(          HMC5883L_DATARATE_15HZ  );
  compass.setSamples(           HMC5883L_SAMPLES_8      );

  /**
   * Dependent on the hardware, needs to be calibrated
   * for each module.
   */
  compass.setOffset( compass_calibration_x, compass_calibration_y );

  /**
   * Set I2C addresses to both displays.
   */
  screen1.setI2CAddress( 0x078 );
  screen2.setI2CAddress( 0x07A );

  screen1.begin();
  screen2.begin();
}

/**
 * Hold old heading value, to reduce refreshes of the compass
 */
uint16_t lastDrawHeading = -100;

void loop() {
  uint16_t heading = getHeading();
  
  if ( ! ( heading > lastDrawHeading - 4 && heading < lastDrawHeading + 4 ) ) {
    screen1.firstPage();
    
    do {
      drawCompass( heading );
      screen1.sendBuffer();
    } while ( screen1.nextPage() );
    
    lastDrawHeading = heading;
  }

  drawInfoScreenText( screen2 );

  delay( 200 );
}

void drawCompass( uint16_t heading ) {
  /**
   * Clear the screen buffer and reset fonts
   */
  screen1.clearBuffer();
  screen1.setDrawColor( 1 );
  screen1.setFont( u8g2_font_victoriamedium8_8r );

  /**
   * Draw compass circle. It's a static circle.
   */
  screen1.drawCircle( compass_centre_x, compass_centre_y, compass_radius, U8G2_DRAW_ALL );

  /**
   * Holds the label that's shown in the middle of
   * the compass circle.
   */
  char compassLabel[ 3 ];

  /**
   * Only draw the compass components when the
   * heading is valid, i.e. when the device is held at ±45º
   */
  if ( isValidHeading( heading ) ) {
    float label_angle;
    uint16_t x2_label, y2_label;

    /**
     * The distance from the compass centre to the label
     */
    uint16_t dirLblDist = compass_radius + 10;

    /**
     * North label
     */
    label_angle = correctAngle( -90 - heading ) * PI / 180.0;
    x2_label = ( compass_centre_x + dirLblDist * cos( label_angle ));
    y2_label = ( compass_centre_y + dirLblDist * sin( label_angle ));
    screen1.drawStr( x2_label - 4, y2_label + 4, "N" );

    /**
     * East label
     */
    label_angle = correctAngle( 0 - heading ) * PI / 180.0;
    x2_label = ( compass_centre_x + dirLblDist * cos( label_angle ));
    y2_label = ( compass_centre_y + dirLblDist * sin( label_angle ));
    screen1.drawStr( x2_label - 4, y2_label + 4, "E" );

    /**
     * South label
     */
    label_angle = correctAngle( 90 - heading ) * PI / 180.0;
    x2_label = ( compass_centre_x + dirLblDist * cos( label_angle ));
    y2_label = ( compass_centre_y + dirLblDist * sin( label_angle ));
    screen1.drawStr( x2_label - 4, y2_label + 4, "S" );

    /**
     * South label
     */
    label_angle = correctAngle( 180 - heading ) * PI / 180.0;
    x2_label = ( compass_centre_x + dirLblDist * cos( label_angle ));
    y2_label = ( compass_centre_y + dirLblDist * sin( label_angle ));
    screen1.drawStr( x2_label - 4, y2_label + 4, "W" );

    /**
     * Draw a tiny pointer arrow to point to the current direction.
     */
    screen1.drawTriangle(
      // top point
      compass_centre_x, ( compass_centre_y - ( compass_radius - compass_pointer_height ) ),
      // left point
      compass_centre_x - compass_pointer_width / 2, ( compass_centre_y - ( compass_radius - compass_pointer_width ) ),
      // right point
      compass_centre_x + compass_pointer_width / 2, ( compass_centre_y - ( compass_radius - compass_pointer_width ) )
    );

    /**
     * Draw the label inside the compass, zero-padded
     */
    sprintf( compassLabel, "%03d", heading );

    /**
     * Draw the heading text on the right of the compass
     */
    // set the font to a large one
    screen1.setFont( u8g2_font_inr38_mr );

    // typecasting magic :(
    char headingTextBuff[ 3 ];
    getHeadingText( heading ).toCharArray(headingTextBuff, 3);

    /**
     * Draw the label, offset at 65 pixels to the right and 52 from top.
     * 
     * Seems u8g2 draws the fonts at a weird offset. Probably font-dependent.
     */
    screen1.drawStr(65, 52, headingTextBuff);
  }
  else {
    /**
     * If the heading is not valid, i.e. device held at ±45°, draw
     * a "LVL" label inside the compass circle.
     */
    sprintf(compassLabel, "LVL");
  }

  /**
   * Draw the label in the middle of the compass circle.
   * 
   * The font size should be chosen to be less than the circle radius.
   * 
   * For example, a circle with a radius of 15 has a diameter of 30.
   * 
   * Font with width 8 can safely fit 3 characters ( 3 * 8 = 24 )
   */

  // reset the font, because it was set to a bigger one above.
  screen1.setFont( u8g2_font_victoriamedium8_8r );

  /**
   * The string coordinates should be calculated for each font.
   */
  screen1.drawStr(
    ( compass_centre_x - floor(8 + 8 / 2) + 1 ),
    ( compass_centre_y + 8 / 2 ),
    compassLabel
  );
}

/**
 * Right-pad a string to 16 characters with spaces.
 * 
 * This is used for the second screen, where the rows are 
 * 16 characters in width.
 * 
 * If this is not used, if a value with longer character length
 * is printed and then a shorter value is printed, there will be
 * artifacts left on the screen.
 * 
 * If the screen or line are cleared beforehand, the screen/line
 * will flicker.
 * 
 * This method prevents flickering by drawing empty spaces until 
 * the end of the line.
 */
String getPaddedString( char * format, uint16_t variable ) {
  char buff[ 16 ];
  sprintf( buff, format, variable );
  sprintf( buff, "%-15s", buff );
  return String( buff );
}

/**
 * Second screen, draw text information from the sensors
 */
void drawInfoScreenText( U8X8_SSD1306_128X64_NONAME_HW_I2C disp ) {
  disp.setFont( u8x8_font_victoriamedium8_r );

  disp.println( getPaddedString( "T: %d *C", round( bmp.readTemperature() ) ) );
  disp.println();

  disp.println( getPaddedString( "P: %d hPa", round( bmp.readPressure() / 100 ) ) );
  disp.println();

  disp.println( getPaddedString( "A: %d m", round( bmp.readAltitude() ) ) );
  disp.println();

  uint16_t heading = getHeading();

  if ( ! isValidHeading( heading ) ) {
    disp.println( "H: Level device" );
  }
  else {
    /**
     * Another typecast hack :(
     */
    char headingFormat[16];
    
    String( 
      String( "H: %d / ") + getHeadingText( heading ) 
      ).toCharArray( headingFormat, sizeof( headingFormat ) );
      
    disp.println( getPaddedString( headingFormat, heading ) );
  }
}

/**
 * Check if the hading is valid. If the inclination is 
 * between 
 */
bool isValidHeading(uint16_t heading) {
  return ! ( heading < 0 || heading > 360 );
}

/**
 * Get heading in degrees.
 */
uint16_t getHeading() {
  Vector mag = compass.readNormalize();
  Vector acc = accelerometer.readScaled();

  float heading = tiltCompensate( mag, acc );

  /**
   * Decrlination angle for Sofia, Bulgaria.
   * 
   * If you're going to use in a dynamic environment 
   * or don't know the angle, the two lines below can
   * be commented out. It will give an error of a few degrees.
   */
  float declinationAngle = ( 4.0 + ( 86.0 / 60.0 ) ) / (180 / M_PI);
  heading += declinationAngle;
  
  heading = correctAngle( heading );
  heading = heading * 180 / M_PI;
  
  return ( uint16_t ) heading;
}

/**
 * Return a correct angle.
 */
float correctAngle( float heading ) {
  if ( heading < 0 ) {
    heading += 2 * PI;
  }
  
  if ( heading > 2 * PI ) {
    heading -= 2 * PI;
  }

  return heading;
}

/**
 * Compensate the compass according to the tilt of the device.
 * 
 * In order to work correctly it requires the device to be kept level
 * between ±45°
 */
float tiltCompensate(Vector mag, Vector normAccel) {
  // Pitch & Roll
  float roll;
  float pitch;

  roll = asin( normAccel.YAxis );
  pitch = asin( -normAccel.XAxis );

  if ( roll > 0.78 || roll < -0.78 || pitch > 0.78 || pitch < -0.78 )
  {
    return -1000;
  }

  // Some of these are used twice, so rather than computing them twice in the algorithem we precompute them before hand.
  float cosRoll = cos( roll );
  float sinRoll = sin( roll );
  float cosPitch = cos( pitch );
  float sinPitch = sin( pitch );

  // Tilt compensation
  float Xh = mag.XAxis * cosPitch + mag.ZAxis * sinPitch;
  float Yh = mag.XAxis * sinRoll * sinPitch + mag.YAxis * cosRoll - mag.ZAxis * sinRoll * cosPitch;

  float heading = atan2( Yh, Xh );

  return heading;
}

/**
 * Get the heading of the device in text
 */
String getHeadingText( uint16_t heading ) {
  if (      heading > 337 || heading < 23  ) return "N";
  else if ( heading > 22  && heading < 68  ) return "NE";
  else if ( heading > 67  && heading < 113 ) return "E";
  else if ( heading > 112 && heading < 158 ) return "SE";
  else if ( heading > 157 && heading < 203 ) return "S";
  else if ( heading > 202 && heading < 248 ) return "SW";
  else if ( heading > 247 && heading < 293 ) return "W";
  else if ( heading > 292 && heading < 338 ) return "NW";
}


