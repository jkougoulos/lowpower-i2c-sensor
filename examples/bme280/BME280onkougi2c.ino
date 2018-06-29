
/*
  ReadAnalogVoltage
  Reads an analog input on pin 0, converts it to voltage, and prints the result to the serial monitor.
  Graphical representation is available using serial plotter (Tools > Serial Plotter menu)
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.
*/

#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/boot.h>


#include <Wire.h>
#include <utility/twi.h>

#include <Adafruit_BME280.h>

#include <RH_RF69.h>
#define MYMTU 60
#define MYTTL 8
RH_RF69 driver;



#define NUM_8S_SLEEP 1

//#define DEBUGSENSOR
//#define DEBUGSENSORTIMING


#define TPL5010_WAKE_PIN 3
#define TPL5010_DONE_PIN 4
#define STM1061_POWER 5
#define STM1061_OUTPUT 6
#define LED 9

typedef struct header {
  uint8_t proto_ver : 4;
  uint8_t proto_type : 4;
  uint16_t ssid;
  uint16_t dsid;
  uint16_t msgseq;
  uint8_t qos : 4;
  uint8_t :4;
  uint8_t ttl;
  uint8_t frg : 4;
  uint8_t tfrg : 4;
  uint16_t mtu : 11;
  uint16_t  : 5;
  uint16_t nexthdr;
} header;

int myID = 998;
char data[255];
byte pos = 0;

unsigned long Cycle = 0;
unsigned long CycleT = 0;
uint16_t msgseq = 0;

Adafruit_BME280 bme;
#define SEALEVELPRESSURE_HPA (1013.25)
#define BME280_I2CADDR 0x76
bool bme_status;

volatile int lowvolt_initial ;
volatile int lowvolt_phase ;
volatile bool lowvolt_send_pend = false;
volatile int phase ;

void checkRFINIT()
{
  if (!driver.init())
  {
#ifdef DEBUGSENSOR
    Serial.println("Driver init failed");
#endif    
  }
  if ( !driver.setFrequency(433.0) )
  {
#ifdef DEBUGSENSOR    
    Serial.println("Radio setFrequency failed");
#endif    
  }
  else
  {
#ifdef DEBUGSENSOR    
    Serial.println("Mote Radio init ok!");
#endif    
  }
  driver.sleep();
  return;
}

void configure_wdt(void)
{
   
    cli();                           // disable interrupts for changing the registers
  
    MCUSR = 0;                       // reset status register flags
  
                                     // Put timer in interrupt-only mode:                                       
    WDTCSR |= 0b00011000;            // Set WDCE (5th from left) and WDE (4th from left) to enter config mode,
                                     // using bitwise OR assignment (leaves other bits unchanged).
//    WDTCSR =  0b01000000 | 0b000110; // 1 sec
//    WDTCSR =  0b01000000 | 0b000111; // 2 sec

    WDTCSR =  0b01000000 | 0b100001; // set WDIE: interrupt enabled
                                     // clr WDE: reset disabled
                                     // and set delay interval (right side of bar) to 8 seconds
    sei();                           // re-enable interrupts 
}

void tpl5010_wake( void )
{
   detachInterrupt(digitalPinToInterrupt(TPL5010_WAKE_PIN));
}

void send_tpl5010_done()
{  
  digitalWrite(TPL5010_DONE_PIN,HIGH);
  delayMicroseconds(1);
  digitalWrite(TPL5010_DONE_PIN,LOW);
  
  return;
}


ISR(PCINT2_vect)
{

   int stm1061_current = digitalRead( STM1061_OUTPUT );
   if ( stm1061_current == LOW )
   {
      lowvolt_send_pend = true;
      lowvolt_phase = phase;
      pciClear( STM1061_OUTPUT );
      digitalWrite( STM1061_POWER, LOW ); // turn off voltage detection to prevent flapping and lower power usage
   }
   else
   {
      lowvolt_phase = 99;
   }
}
  
ISR(WDT_vect)
{
  
     if( 1 )
     {
        wdt_reset();
     }
     else
     {
        // must be rebooted
        // configure
        MCUSR = 0;                          // reset flags
         
                                              // Put timer in reset-only mode:
        WDTCSR |= 0b00011000;               // Enter config mode.
        WDTCSR =  0b00001000 | 0b000000;    // clr WDIE (interrupt enable...7th from left)
                                              // set WDE (reset enable...4th from left), and set delay interval
                                              // reset system in 16 ms...
                                              // unless wdt_disable() in loop() is reached first
                                         
        // reboot
        while(1);
     }
}


void sleepNow(int num8s = 1)         // here we put the arduino to sleep
{
    for( int i = 0 ; i < num8s ; i++ )
    {
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
        cli();

        attachInterrupt( digitalPinToInterrupt(TPL5010_WAKE_PIN), tpl5010_wake , RISING );

        sleep_enable();
        sleep_bod_disable();
        sei();
        sleep_cpu();
        sleep_disable();
        sei();
    }

}


void WDT_off(void)
{
  cli(); // __disable_interrupt();
  wdt_reset(); //__watchdog_reset();
  /* Clear WDRF in MCUSR */
  MCUSR &= ~(1<<WDRF);
  /* Write logical one to WDCE and WDE */
  /* Keep old prescaler setting to prevent unintentional
  time-out */
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  /* Turn off WDT */
  WDTCSR = 0x00;
  sei(); //__enable_interrupt();
}

  
// the setup routine runs once when you press reset:
void setup() {

  int oldMCUSR = MCUSR;
  
  MCUSR = 0;
  // initialize serial communication at 9600 bits per second:
#if defined(DEBUGSENSOR) || defined(DEBUGSENSORTIMING)
  Serial.begin(9600);
  
  Serial.print("Hallo! MCUSR was:");
  Serial.println( oldMCUSR );


    cli();
    uint8_t lowBits      = boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
    uint8_t highBits     = boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
    uint8_t extendedBits = boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);
    uint8_t lockBits     = boot_lock_fuse_bits_get(GET_LOCK_BITS);
    sei();

    Serial.print("Fuses: ");
    Serial.print("Low: 0x");
    Serial.print(lowBits, HEX);
    Serial.print(" High: 0x");
    Serial.print(highBits, HEX);
    Serial.print(" Ext: 0x");
    Serial.print(extendedBits, HEX);
    Serial.print(" Lock: 0x");
    Serial.println(lockBits, HEX);

    char tmpstr[3];
    Serial.print("Signature: 0x");
    for (uint8_t i = 0; i < 5; i += 2) {
//        Serial.print("i:") ; Serial.print(i); Serial.print(" ");
        
//        Serial.println(boot_signature_byte_get(i), HEX);
        sprintf(tmpstr,"%02X",boot_signature_byte_get(i));
        Serial.print(tmpstr);
    }
    Serial.println();

    
    Serial.print("Serial Number: 0x");
    for (uint8_t i = 14; i < 24; i += 1) {
//        Serial.print(" 0x");
//        Serial.print(i); Serial.print(":");
        sprintf(tmpstr,"%02X",boot_signature_byte_get(i));
//        Serial.println(boot_signature_byte_get(i), HEX);
        Serial.print(tmpstr);
    }
    Serial.println();
#endif  
 
  WDT_off();

#ifdef DEBUGSENSOR
  configure_wdt();
#endif


  DDRB =  0b00000010 ; // 0x00;        //make port a as input except pin 9 / B1 / LED
  PORTB = 0b11111101 ; // 0xFF;       // with pullup

  DDRC = 0x7F;        // make port C as input
  PORTC = 0x7F;       // with pullup

  DDRD =  0b00110000; //0x00;        // make port D as input except PD4 / TPL5010 DONE
  PORTD = 0b01000111; // 0xFF;       // with pullup except PD3 / 3 / TPL5010 WAKE / 5,6 STM / 7 RFM-RST

  DDRE = 0x00;        //make port a as input
  PORTE = 0xF;       // with pullup (4 bit port)

  ADCSRA = 0;  // need to shutdown ADC before disabling
  
  PRR1 = (1 << PRTWI1) | (1 << PRTIM4) | (1 << PRSPI1) | (1 << PRPTC) | (1 << PRTIM3);
//  PRR0 = (1 << PRTIM2) | (1 << PRTIM0) | (1 << PRTIM1) | (1 << PRTWI0) | (1 << PRUSART1) | (1 << PRUSART0) | (1 << PRADC) | (1 << PRSPI0);
  PRR0 =  (1 << PRTIM2)  | (1 << PRTIM1) | (1 << PRUSART1) | (1 << PRADC)    // we need timer0 for delay()/micros() etc. SPI0 for RFM, TWI0 for sensor
#if !defined(DEBUGSENSOR) && !defined(DEBUGSENSORTIMING)
          | (1 << PRUSART0 )    // shutdown serial when not debugging
#endif
          ;          

   pinMode(LED, OUTPUT);
   digitalWrite(LED,LOW);
   
   pinMode(TPL5010_WAKE_PIN, INPUT);
   pinMode(TPL5010_DONE_PIN, OUTPUT);
   send_tpl5010_done(); // startup TPL5010; otherwise resistor for setting delay consumes power 
   
   pinMode(STM1061_POWER,OUTPUT);
   digitalWrite(STM1061_POWER, LOW);
   pinMode(STM1061_OUTPUT,INPUT_PULLUP);

    
    // default settings
    // (you can also pass in a Wire library object like &Wire2)
   bme_status = bme.begin( BME280_I2CADDR );  
   if (!bme_status) {
#if defined(DEBUSENSOR) || defined(DEBUGSENSORTIMING)    
       Serial.println("Could not find a valid BME280 sensor, check wiring!");
#endif
       while (1);  // wait for watchdog to reset us
   }

/* twi bit rate formula from atmega128 manual pg 204
  SCL Frequency = CPU Clock Frequency / (16 + (2 * TWBR))
  note: TWBR should be 10 or higher for master mode
  It is 72 for a 16mhz Wiring board with 100kHz TWI */
#if F_CPU == 1000000        //koug
   TWBR = 0;  // get MAX speed on I2C bus (62.5Khz in 1Mhz CPU)
#endif

   checkRFINIT();
  
   
}

void transmitInfo()
{
  int data_size = strlen(data);

  if( driver.send( (uint8_t *)data, data_size ) )
  {
      //sleepNow(1);   // power down cpu while transmitting 
      driver.waitPacketSent();
#ifdef DEBUGSENSOR
      Serial.println( "###\nData sent!");
      delay(200);
#endif
  }
#ifdef DEBUGSENSOR
  else
  {
      Serial.println( "###\nData NOT sent!");
      delay(100);
  }
#endif
  return;  
}



void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

void pciClear(byte pin)
{
    *digitalPinToPCMSK(pin) &= ~(bit (digitalPinToPCMSKbit(pin)));  // disable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  &= ~(bit (digitalPinToPCICRbit(pin))); // disable interrupt for the group
}

void lowvolt_detect_activate()
{
  
  digitalWrite(STM1061_POWER, HIGH);
  delayMicroseconds(200);  // wait for STM to stabilize

  lowvolt_initial = digitalRead( STM1061_OUTPUT ) ;
  if( lowvolt_initial == LOW ) // low voltage detected
  {
#ifdef DEBUGSENSOR
    Serial.print("STML ");
#endif    
    lowvolt_send_pend = true;
  }
  else         // HIGH therefore voltage now is ok, attaching interrupt to monitor changes
  {
#ifdef DEBUGSENSOR
    Serial.print("STMH ");
#endif    
    pciSetup( STM1061_OUTPUT );
  }
 }

void add_tag_to_data(const char *tag,byte taglen, void *val,bool islong)
{
    
    
    strcpy( data+pos, tag );
    pos += taglen;
    
    if( islong )
    {
      ltoa( *(long *)val, data+pos,16);
    }
    else
    {
      itoa( *(int*)val, data+pos,16);
    }
    pos = pos + strlen(data+pos);

    return;
}




// the loop routine runs over and over again forever:
void loop() {


  phase = 0;

#ifdef DEBUGSENSORTIMING  
  unsigned long t0 = micros();
#endif
  
  
  Cycle++;

  digitalWrite(LED, HIGH);
  delayMicroseconds(1000);
  digitalWrite(LED, LOW);


//  if( Cycle == 3 )  { while(1) {}; }        // test watchdog reset
  
  
  phase = 1;
  
  if( !lowvolt_send_pend )
  {
    lowvolt_detect_activate();
  }
 
  phase = 10;
#ifdef DEBUGSENSORTIMING
  unsigned long t10  = micros() - t0;
  unsigned long t14 = 0;
  unsigned long t15 = 0;
  unsigned long t16 = 0;
  unsigned long t17 = 0;
#endif


 // int but multiplied by 10 to keep the first decimal digit
  int temp_int = 0;
  int hum_int = 0;
  int pressure_int = 0 ;
  
  if( bme_status )  
  {
    bme.takeForcedMeasurement();

#ifdef DEBUGSENSORTIMING
    phase = 14;
    t14  = micros() - t0;
#endif


//    bme.readPTHFast( &pressure, &temp, &hum );
   bme.readPTHFast( &pressure_int, &temp_int, &hum_int );
}
  
  phase = 18;
#ifdef DEBUGSENSORTIMING
  unsigned long t18  = micros() - t0;
#endif


  unsigned long currmicros = micros();
  CycleT = currmicros - CycleT;
  
 
    pos = 0;

//    sprintf(data,"<ID0:%d/MS0:%ld/C00:%ld/T01:%d/H01:%d/P01:%d>\n",myID,millis(),Cycle,temp_int,hum_int,pressure_int);
//    sprintf(data,"<ID0:%d/MS0:%ld/C00:%ld/T01:%d/H01:%d/P01:%d>\n",myID,CycleT,Cycle,temp_int,hum_int,pressure_int);

    data[pos++] = '<';

    add_tag_to_data("ID1:", 4, &myID, false);
    add_tag_to_data("/MS0:", 5, &CycleT, true);
    add_tag_to_data("/C00:", 5, &Cycle, true);
    add_tag_to_data("/T01:", 5, &temp_int, false);
    add_tag_to_data("/H01:", 5, &hum_int, false);
    add_tag_to_data("/P01:", 5, &pressure_int, false);

 

  if( lowvolt_send_pend )
  {
    add_tag_to_data("/BAT:", 5, &lowvolt_phase, false);

//    sprintf(data,"<ID0:%d/MS0:%ld/C00:%ld/T01:%d/H01:%d/P01:%d/BAT:L%d>\n",myID,CycleT,Cycle,temp_int,hum_int,pressure_int,lowvolt_phase);
    lowvolt_send_pend = false;
    lowvolt_phase = 0;
    lowvolt_detect_activate();
  }
  else
  {
    data[pos++] = '/';
    data[pos++] = 'B';
    data[pos++] = 'A';
    data[pos++] = 'T';
    data[pos++] = ':';
    data[pos++] = 'O';
    data[pos++] = 'K';
  }

  data[pos++] = '>';
  data[pos++] = '\n';
  data[pos++] = '\0';

//    Serial.println(data);

  
  CycleT = currmicros;

  phase = 19;

#ifdef DEBUGSENSORTIMING
  unsigned long t19  = micros() - t0;
#endif
  
  transmitInfo();
  driver.sleep();

  phase = 20;

#ifdef DEBUGSENSORTIMING
  unsigned long t20  = micros() - t0;
#endif

  digitalWrite(LED, HIGH);
  delayMicroseconds(1000);
  digitalWrite(LED, LOW);

  phase = 25;
#ifdef DEBUGSENSORTIMING
  unsigned long t25  = micros() - t0;
#endif

  phase = 30;
#ifdef DEBUGSENSORTIMING
  unsigned long t30  = micros() - t0;
#endif
  
  send_tpl5010_done();

  phase = 40;
#ifdef DEBUGSENSORTIMING
  unsigned long t40  = micros() - t0;
#endif

  pciClear( STM1061_OUTPUT );
  digitalWrite(STM1061_POWER, LOW);

#ifdef DEBUGSENSORTIMING
  Serial.print("t0: ");
  Serial.println(t0);
  Serial.print("t10: ");
  Serial.println(t10);
  Serial.print("t14: ");
  Serial.println(t14);
  Serial.print("t15: ");
  Serial.println(t15);
  Serial.print("t16: ");
  Serial.println(t16);
  Serial.print("t17: ");
  Serial.println(t17);
  Serial.print("t18: ");
  Serial.println(t18);
  Serial.print("t19: ");
  Serial.println(t19);
  Serial.print("t20: ");
  Serial.println(t20);
  Serial.print("t25: ");
  Serial.println(t25);
  Serial.print("t30: ");
  Serial.println(t30);
  Serial.print("t40: ");
  Serial.println(t40);
 /*
  Serial.print( "BAT LOW @ " );
  Serial.println( lowvolt_phase );
  Serial.println(temp_int);
  Serial.println(hum_int);
  Serial.println(pressure_int);
  */
  Serial.println("<---------->");

  delay(100);
#endif

  //delay(5000);
  sleepNow(NUM_8S_SLEEP);
 
}
