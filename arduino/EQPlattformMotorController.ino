// Autor: Sebastian Förster
// License: MIT

#define CONTROLLER_SPEED    8 // 4 = 1/4 second - should be a divider of 1000 ms
#define GEAR_RATIO          134
#define TIMINGBELT_RATIO    2
#define ENCODER_TICKS_RES   30
#define SPEED_SETPOINT      (34.0f / 3600.0f * GEAR_RATIO * ENCODER_TICKS_RES * TIMINGBELT_RATIO / CONTROLLER_SPEED)
//SPEED_SETPOINT = 75.93333


const int ledPin =  LED_BUILTIN;// the number of the LED pin

// These constants won't change. They're used to give names to the pins used:
const int mot_forward = 9;
const int mot_backward = 10;

const int sensorPin = A0;    // select the input pin for the potentiometer

const int platformend_east = A2;
const int platformend_west = A1;

const int guiding_output_north = 4;
const int guiding_output_south = 5;

volatile unsigned int encoder_cnts = 0;

unsigned long move_axis_timer_de = 0;
unsigned long move_axis_timestamp_de = 0;
unsigned long move_axis_timer_ra = 0;
unsigned long move_axis_timestamp_ra = 0;
char pulse_direction_de = ' ';
char pulse_direction_ra = ' ';

void setup_fastpwm(void)
{
  TCCR1A = 0;
  TCCR1B = 0;

  //Modus Fast PWM-Mode 10 Bit einstellen
  TCCR1A |= (1 << WGM10) | (1 << WGM11);
  TCCR1B |= (1 << WGM12);

  //Vorteiler auf 8 setzen
  //TCCR1B |= (1 << CS11);

  //keine Teilung
  TCCR1B |= (1 << CS10);

  //Nichtinvertiertes PWM-Signal setzen
  TCCR1A |= (1 << COM1A1);
  TCCR1A |= (1 << COM1B1);

  //PWM-Pin 9 als Ausgang definieren
  DDRB |= (1 << DDB1);
  //PWM-Pin 10 als Ausgang definieren
  DDRB |= (1 << DDB2);
}

void fast_analogWrite(int pin, int pwm)
{
  if(pin == mot_forward)
  {
    OCR1A = pwm >> 1; //driver slower
  }
  if(pin == mot_backward)
  {
    OCR1B = pwm << 2; //convert to from 8 bit to 10 bit
  }
}


int pid_control(float setpoint, float value)
{
 
  static float I = 0.4f;
  static float P = 6.0f;
  static float D = 0.0f;
  static float err_sum = 0.0f;
  static float last_error = 0.0f;
  
  float error = setpoint - value;
  err_sum += error; // I sum

  float output = P*error + I*err_sum + D * (error - last_error);

  last_error = error;
  
  if(output >= 255.0f)
    return 255;
  if(output <= 0.0f)
    return 0.0;

  return (int)output;
}

void EncoderISR(void)
{
  encoder_cnts++;
}

String lx200RA  = "00:00:00#";
String lx200DEC = "+90*00:00#";

String mount_name = "eqplat";
const unsigned long firmware_version = 220415;


// all :.*# commands are passed here 
void lx200(String s) 
{ 
  // :GR# 
  if (s.substring(1,3).equals("GR")) 
  { 
    Serial.print(lx200RA);
  } 
  // :GD# 
  else if (s.substring(1,3).equals("GD")) 
  { 
    Serial.print(lx200DEC);
  } 
  // :GV*# Get Version *
  else if (s.substring(1,3).equals("GV")) 
  { 
    char c = s.charAt(3); 
    if ( c == 'P') {// GVP - Product name
       Serial.print(mount_name);  
    } else if (c == 'N') { // GVN - firmware version
       Serial.print(firmware_version);  
    }
    Serial.print('#');
  } 
  // :SrHH:MM:SS# or :SrHH:MM.T# // no blanks after :Sr as per Meade specss
  else if (s.substring(1,3).equals("Sr")) 
  {
    // this is INITAL step for setting position (RA)
    long hh = s.substring(3,5).toInt();
    long mi = s.substring(6,8).toInt();
    long ss = 0;
    // :SrHH:MM.T#
    if (s.charAt(8) == '.') 
    { 
      ss = (s.substring(9,10).toInt())*60/10;
    } 
    else 
    {
      ss = s.substring(9,11).toInt();
    }
    long inRA = hh*3600+mi*60+ss;
    //copy string as new reference
    lx200RA = s.substring(3);
    Serial.print(1); // FIXME: input is not validated
  } 
  // :SdsDD*MM:SS# or :SdsDD*MM#
  else if (s.substring(1,3).equals("Sd")) 
  {
    // this is the FINAL step of setting a pos (DEC) 
    long dd = s.substring(4,6).toInt();
    long mi = s.substring(7,9).toInt();
    long ss = 0;
    if (s.charAt(9) == ':') 
    { 
      ss = s.substring(10,12).toInt(); 
    }
    long inDEC = (dd*3600+mi*60+ss)*(s.charAt(3)=='-'?-1:1);
    //copy string as new reference
    lx200DEC = s.substring(4);
    Serial.print(1); // FIXME: input is not validated
  }
  // MOVE:  :MS# (slew), :Mx# (slow move)
  // :Mgn1000\n slew one second north
  else if (s.charAt(1) == 'M') 
  { 
    // SLEW
    if (s.charAt(2) == 'S' ) 
    {
      
    }
    //pulse guiding
    else if(s.charAt(2) == 'g' )
    {
      char pulse_direction = s.charAt(3);
      if(pulse_direction == 'n' || pulse_direction == 's')
      {
        move_axis_timer_de = s.substring(4,8).toInt();
        move_axis_timestamp_de = millis();
        pulse_direction_de = pulse_direction;
      }
      else if(pulse_direction == 'w' || pulse_direction == 'e')
      {
        move_axis_timer_ra = s.substring(4,8).toInt();
        move_axis_timestamp_ra = millis();
        pulse_direction_ra = pulse_direction;
      }
    }
    else 
    {
      switch (s.charAt(2)) 
      {
        case 'n':
          //moveDecNorth();
          break;
        case 's':
          //moveDecSouth();
          break;
        case 'w':
          //moveRaWest();
          break;
        case 'e':
          //moveRaEast();
          break;
      }
    }
  } 
  // :Q# or :Qx# stop Dec Motor and set RA to Tracking
  else if (s.charAt(1) == 'Q') 
  { 
    //moveDecHalt();
    //moveRaTracking();
  } 
  // :CM# sync
  else if (s.substring(1,3).equals("CM")) 
  { 
    // assumes Sr and Sd have been processed
    // sync current position with input
    //currRA  = inRA;
    //currDEC = inDEC;
    Serial.print("Synced#");
  }
  else
  {
    //dummy for faster response to not implemented commands
    Serial.print('#');
  }
}

void read_serial_lx200_in(void)
{
  static char input[20];
  static uint8_t in_cnt = 0;
  
  // Check if message on serial input
  if (Serial.available() > 0) {
    char curr_char = Serial.read();
    input[in_cnt] = curr_char;

    // discard blanks. Meade LX200 specs states :Sd and :Sr are
    // not followed by a blank but some implementation does include it.
    // also this allows aGoto commands to be typed with blanks
    if (curr_char == ' ') return; 
    
    // acknowledge LX200 ACK signal (char(6)) for software that tries to autodetect protocol (i.e. Stellarium Plus)
    if (curr_char == char(6)) 
    { 
      Serial.print("P");  // P = Polar
      return; 
    }

    if (curr_char == '#' || curr_char == '\n') { // after a # or a \n it is time to check what is in the buffer
      if (input[0] == ':') // lx200 protocol
      { 
        lx200(input);
      }
      in_cnt = 0; // reset buffer
    } 
    else {
      in_cnt++;
      if (in_cnt >= sizeof(input))
        in_cnt = 0; // reset buffer - message dropped
    } 
  }
}

void setup() {
  Serial.begin(9600);
  setup_fastpwm();

  pinMode(2, INPUT); 
  attachInterrupt(0, EncoderISR, CHANGE); //pin 2!!

  //Pin 12 used as GND supply
  pinMode(12, OUTPUT); 
  digitalWrite(12, LOW);

  //use for end switches
  pinMode(platformend_east, INPUT_PULLUP);
  pinMode(platformend_west, INPUT_PULLUP);

  //use for guiding of the DE axis
  pinMode(guiding_output_north, OUTPUT);
  pinMode(guiding_output_south, OUTPUT);
  
  //Pin 8 & 11 used as 5V supply
  pinMode(11, OUTPUT); 
  digitalWrite(11, HIGH);
  pinMode(8, OUTPUT); 
  digitalWrite(8, HIGH);

  pinMode(ledPin, OUTPUT);
}

unsigned long currentTime = 0, previousTime = 0;
unsigned int encoder_old_ticks = 0, encoder_current_ticks = 0;
float setpoint_offset = 0.0f;
float mean_ticks = 0.0f;
int16_t diff_list[CONTROLLER_SPEED];
int16_t diff_index = 0;

bool west_switch_active = false;


void loop() {

  int sensorvalue = analogRead(sensorPin);

  read_serial_lx200_in();

  if(move_axis_timer_ra > 0)
  {
    if(millis() - move_axis_timestamp_ra >= move_axis_timer_ra)
    {
      move_axis_timer_ra = 0;
      setpoint_offset = 0.0f;

      if(move_axis_timer_de == 0)
        digitalWrite(ledPin, LOW);
    }
    else
    {
      digitalWrite(ledPin, HIGH);
      if(pulse_direction_ra == 'w')
        setpoint_offset = SPEED_SETPOINT * 0.5;
      else if(pulse_direction_ra == 'e')
        setpoint_offset = SPEED_SETPOINT * -0.5f;
    }
  }

  if(move_axis_timer_de > 0)
  {
    if(millis() - move_axis_timestamp_de >= move_axis_timer_de)
    {
      move_axis_timer_de = 0;
      digitalWrite(guiding_output_north, LOW);
      digitalWrite(guiding_output_south, LOW);

      if(move_axis_timer_ra == 0)
        digitalWrite(ledPin, LOW);
    }
    else
    {
      digitalWrite(ledPin, HIGH);
      if(pulse_direction_de == 'n')
        digitalWrite(guiding_output_north, HIGH);
      else if(pulse_direction_de == 's')
        digitalWrite(guiding_output_south, HIGH);
    }
  }

  //stop via poti has the highest prio
  if(sensorvalue > 255 && sensorvalue < (1023 - 255))
  {
    west_switch_active = false;
    //stop platform
    fast_analogWrite(mot_forward, 0);
    fast_analogWrite(mot_backward, 0);

    //reset encoder ticks
    cli();
    encoder_old_ticks = encoder_cnts;
    sei();

    return;
  }

  //if east switch is reached
  if(digitalRead(platformend_west) == HIGH)
    west_switch_active = true;

  if(west_switch_active)
  {  
    if(digitalRead(platformend_east) == HIGH)
    {
      west_switch_active = false;
      //stop platform
      fast_analogWrite(mot_forward, 0);
      fast_analogWrite(mot_backward, 0);

      //reset encoder ticks
      cli();
      encoder_old_ticks = encoder_cnts;
      sei();
    }
    else
    {
      fast_analogWrite(mot_backward, 255);
      fast_analogWrite(mot_forward, 0);
    }

    return;
  }


  
  //use this range to start star speed controlled movment of dc motor
  if(sensorvalue >= (1023 - 255))
  {
    currentTime = millis();
    if( (currentTime - previousTime) >= 1000/CONTROLLER_SPEED)
    {
      cli();
      encoder_current_ticks = encoder_cnts;
      sei();
      previousTime = currentTime;
  
      unsigned int diff = encoder_current_ticks - encoder_old_ticks;
      diff_list[diff_index] = diff;
      diff_index++;
      if(diff_index >=  CONTROLLER_SPEED)
        diff_index = 0;
      encoder_old_ticks = encoder_current_ticks;
  
      if(mean_ticks == 0.0)
        mean_ticks = diff;
  
      float alpha = 0.5f;
      mean_ticks = (float)diff * ( 1.0f - alpha) + mean_ticks * alpha;

      int diff_res = 0;
      for(int i = 0; i < CONTROLLER_SPEED; i++)
      {
        diff_res += diff_list[i];
      }
  
      int out = pid_control(SPEED_SETPOINT + setpoint_offset, mean_ticks);
      //Serial.print(diff_res);
      //Serial.print(",");
      //Serial.println(out);
      
      fast_analogWrite(mot_forward, out << 1);
      
    }
    //fast_analogWrite(mot_forward, sensorvalue -(1023 - 255));
    fast_analogWrite(mot_backward, 0);
  }
  //move plattform back in variable speed
  else if(sensorvalue <= (255))
  {
    if(digitalRead(platformend_east) == LOW)
    {
      fast_analogWrite(mot_backward, 255 - sensorvalue);
      fast_analogWrite(mot_forward, 0);
    }
    else
    {
      fast_analogWrite(mot_forward, 0);
      fast_analogWrite(mot_backward, 0);
    }
    
    cli();
    encoder_old_ticks = encoder_cnts;
    sei();
  }
}
