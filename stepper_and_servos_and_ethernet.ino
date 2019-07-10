#define F_CPU 16000000UL

#include <Servo.h>
Servo servo_fetcher;
Servo servo_returner;
//#define DEBUGSERVO
#include <SPI.h>
#include <Ethernet.h>
//#define DEBUGSTEP
//#define DEBUGGAME
#define DEBUGSELECT
#define SOCKETPORT 23
#define STEPPEROFFSET 3
#define STEPPERSPEED 35


//------------------structs and variables----------------------
//stepper struct for variables
struct stepperMot{
  int stepperstate; //1 to 4, determins pin-output
  int is_working; //
  unsigned long old_steppertime; //important for delay time
  volatile int current_pos; // 
  int plant_pos[4]; //positions of fout plants, plant_pos[0] represents Plant 1
  int is_calibrated; // important at startup
};
//initialize struct for turntable
volatile struct stepperMot stepper1;

//struct for servo motor variables
struct servoMot{
  int current_pos; //current position
  int target_pos; //target position to push the plant
  int idle_pos; //satrting and idle position
  int state; //state, determins whether to move backward or forward
  Servo connected_servo; //connected servo
};
//initialize 2 structs for both arms
struct servoMot fetcher_struct;
struct servoMot returner_struct;

struct gameStruct{
  int plant_to_be_fetched; //triggers servo
  int torture_table; //is a plant on torture table?
};
struct gameStruct game;


//mac adress of ethernet shield
byte mac[] = {
  0x90, 0xA2, 0xDA, 0x00, 0x7B, 0x60 };
//ip adress, first 3 numbers of gateway, the last one random
//Lab
IPAddress ip(10, 90, 1, 170);
IPAddress myDns(192, 168, 1, 1);
//copy from ipconfig
IPAddress gateway(10, 90, 1, 251);
IPAddress subnet(255, 255, 255, 0);
//Tobi
/*IPAddress ip(192, 168, 0, 20);
IPAddress myDns(192, 168, 1, 1);
//copy from ipconfig
IPAddress gateway(192, 168, 0, 251);
IPAddress subnet(255, 255, 255, 0);*/

//Ethernet Server
EthernetServer server(SOCKETPORT);
//array for up to 8 clients (more are not supported by the shield)
EthernetClient clients[8];

//simplified variables
//EthernetClient client;
boolean alreadyConnected = false;
//-----------------------------------------------------



//variables for user input
bool plant_selected = false; //for serial input can be removed, i guess
int selected_plant = 0;


//-----------------prototype functions----------------------
//stepper prototype functions
//turn motor with (struct motor, int position, int delaytime)
int stepperSteps(struct stepperMot, int, int);
//move to zero position
void calibrate(struct stepperMot);
//setup variables for turntable
void initialize_stepper();

//servo prototype functions
//move selected servo to target and idlepos
void move_servo(struct servoMot);
//move plant to torture table
int fetch_plant();
//move plant to turntable
int return_plant();
//setup variables for both arms
void initialize_servos();
//move to idlepos
void move_servos_to_idle(struct servoMot, struct servoMot);

//game prototype functions
void initialize_game();

//ethernet prototype functions
void initialize_ethernet();
void stop_disconnected_clients();
void add_new_client();
void read_from_client();
void select_plant(int);
void write_to_all_clients(char);
//---------------------------------------------------------


//########################################################
//########################################################
void setup() {
  //setup serial, test variables
  Serial.begin(9600);
  initialize_ethernet();
  initialize_stepper();
  initialize_servos();
  initialize_game();
  calibrate(&stepper1);
}
void loop() {
  //Serial.println("test");
  //add_new_client();
  read_from_client();

  //turn stepper to selected plant position
  while(stepper1.is_working && stepper1.is_calibrated == 1){
    //move stepper 1 to destination with speed 40
    stepper1.is_working = stepperSteps(&stepper1, stepper1.plant_pos[selected_plant-1], STEPPERSPEED);
    if(!stepper1.is_working && !game.torture_table){
      game.plant_to_be_fetched = 1;
      
    }
    #ifdef DEBUGSTEP
    Serial.print("current pos: ");
    Serial.println(stepper1.current_pos);
    #endif
  }

  //trigger servo arm
  if(game.plant_to_be_fetched){
    game.plant_to_be_fetched = 0;
    game.torture_table = 1;
    fetch_plant();
    #ifdef DEBUGGAME
    Serial.println("fetching my plant");
    #endif
    write_to_all_clients('r'); //stupid method
    //server.write('r'); //should only be sent to one client
  }

  //debugstuff for stepper
  #ifdef DEBUGSTEP
  //current pos
  Serial.print("current pos: ");
  Serial.println(stepper1.current_pos);
  delay(100);
  #endif
  
}
//########################################################
//########################################################


//-------------------ethernet methods--------------------------------
void initialize_ethernet(){
  #ifdef DEBUGSELECT
  Serial.println("initializing ethernet");
  #endif
  
  //pin of standard ethernet shield
  Ethernet.init(10);
  // initialize ethernet device
  Ethernet.begin(mac, ip, myDns, gateway, subnet);
  
  // Check connected hardware
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("No ethernet shield found.");
    while (true) {
      delay(1); // abort, loop infinitely
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }

  //start server, listen for clients
  server.begin();
  Serial.print("Socket server address:");
  Serial.println(Ethernet.localIP());
  Serial.print("port:");
  Serial.print(SOCKETPORT);
}


void read_from_client(){
    EthernetClient client = server.available();
     #ifdef DEBUGSELECT
     //Serial.println("trying to read from clients");
     #endif
    if (client) {
      if (!alreadyConnected) {
        // clear out the input buffer:
        client.flush();
        Serial.println("We have a new client");
        alreadyConnected = true;
      }

    if (client.available() > 0) {
      // read the bytes incoming from the client:
      char thisChar = client.read();
      // echo the bytes back to the client:

      // echo the bytes to the server as well:
      
      Serial.print(thisChar);

      if (thisChar == '1'){
            select_plant(1);
          } else if(thisChar == '2'){
            select_plant(2);
          } else if (thisChar == '3'){
            select_plant(3);
          } else if(thisChar == '4'){
            select_plant(4);
          } else {
            #ifdef DEBUGSELECT
            Serial.println("Client is stupid");
            #endif
      }
    }   
  }
}

void write_to_all_clients(char c){
  server.write(c);
}

//return old plant if necessary and start stepper
void select_plant(int i){
  #ifdef DEBUGSELECT
  Serial.print("Plant ");
  Serial.print(i);
  Serial.println(" selected");
  #endif

  stepper1.is_working = 1;
  
  if(game.torture_table){
  
    #ifdef DEBUGGAME
    Serial.println("returning plant");
    #endif
    
    return_plant();
    game.torture_table = 0;
  }

  selected_plant = i;
}

//-------------------------------------------------------------------


//---------------------stepper methods--------------------------------
int stepperSteps(struct stepperMot *mot, int pos, int del){
//-------------jo, maybe !=pos-1 || !=pos || !=pos+1-------------------------
  if (mot->current_pos != pos){ 
      #ifdef DEBUGSTEP
      Serial.print("test");
      #endif
      //if delaytime is reached move one step
      if (millis() > mot->old_steppertime){
      switch(mot->stepperstate){
        case 0: PORTC |= (1<<PC0);
                PORTC &= ~(1<<PC1);
                PORTC |= (1<<PC2);
                PORTC &= ~(1<<PC3);
                mot->stepperstate++;
                break;
        case 1: PORTC &= ~(1<<PC0);
                PORTC |= (1<<PC1);
                PORTC |= (1<<PC2);
                PORTC &= ~(1<<PC3);
                mot->stepperstate++;
                break; 
        case 2: PORTC &= ~(1<<PC0);
                PORTC |= (1<<PC1);
                PORTC &= ~(1<<PC2);
                PORTC |= (1<<PC3);
                mot->stepperstate++;
                break; 
        case 3: PORTC |= (1<<PC0);
                PORTC &= ~(1<<PC1);
                PORTC &= ~(1<<PC2);
                PORTC |= (1<<PC3);  
                mot->stepperstate = 0;
                break; 
      }
      mot->current_pos++;
      mot->old_steppertime = millis() + del;
    }
    return 1;
  } else {
    return 0;
  }
}

void calibrate(struct stepperMot *mot){
  #ifdef DEBUGSTEP
  Serial.println("tries to calibrate");
  #endif
  
  mot->is_working = 1;
  //move stepper until the interrupt is triggered
  while((mot->current_pos) > 0){
    //set destination of stepper to infinity to keep turning
    mot->is_working = stepperSteps(mot, 65535, STEPPERSPEED);
    #ifdef DEBUGSTEP
    Serial.print("calibrating, current pos: ");
    Serial.println(mot->current_pos);
    #endif
  }
  mot->is_working = 0;//maybe delete, for servo test
  write_to_all_clients('r'); //stupid
  stepper1.is_calibrated = 1;
}

void initialize_stepper(){
  //setup stepper1 struct
  stepper1.stepperstate = 0;
  stepper1.is_working = 0;
  stepper1.old_steppertime = 0;
  stepper1.is_calibrated = 0;
  //initialize plant- and current position
  stepper1.current_pos = 200; //infinite
  stepper1.plant_pos[0] = 120+STEPPEROFFSET; //plant 1
  stepper1.plant_pos[1] = 170+STEPPEROFFSET;
  stepper1.plant_pos[2] = 20+STEPPEROFFSET;
  stepper1.plant_pos[3] = 70+STEPPEROFFSET; //plant 4
  
  //initialize analog in pins 0 to 3 as outputs for stepper motor
  DDRC |= (1 << PC0);
  DDRC |= (1 << PC1);
  DDRC |= (1 << PC2);
  DDRC |= (1 << PC3);

  //use pin 3 as interrupt pin with vector 1 and external pulldown
  pinMode(3, INPUT);
  attachInterrupt(1, pin_ISR, FALLING);
}
//-------------------------------------------------------------------



//----------------------servo methods--------------------------------
int fetch_plant(){
  move_servo(&fetcher_struct);
  return 1;
}

int return_plant(){
  move_servo(&returner_struct);
  return 1;
}

//move servo from idle position to destination and back
void move_servo(struct servoMot *serv){
  //check if you have to move forward or backward
  if (serv->target_pos > serv->idle_pos){
    //move to destination
    int i = serv->current_pos;
    
    #ifdef DEBUGSERVO
    Serial.print("i should be current pos: ");
    Serial.println(i);
    #endif
    
    while(!(serv->state)){
      if (serv->current_pos == (serv->target_pos)-1){
        serv->state = 1;
      }
      (serv->connected_servo).write(i);
      
      #ifdef DEBUGSERVO
      Serial.print("position i: ");
      Serial.println(i);
      Serial.print("state: ");
      Serial.println(serv->state);
      #endif
      
      serv->current_pos = i;
      i++;
      delay(20);
      
    }
    i = serv->current_pos;
    //return to idle position
    while (serv->state){
      if (serv->current_pos == (serv->idle_pos)+1){
        serv->state = 0;
      }
      (serv->connected_servo).write(i);
      
      #ifdef DEBUGSERVO
      Serial.print("position i: ");
      Serial.println(i);
      Serial.print("target_pos_fetcher: ");
      Serial.println(serv->target_pos);
      #endif
      
      serv->current_pos = i;
      i--;
      delay(10);
    }
  } else {
    int i = serv->current_pos;
    while(!(serv->state)){
      if (serv->current_pos == (serv->target_pos)+1){
        serv->state = 1;
      }
      (serv->connected_servo).write(i);
      serv->current_pos = i;
      i--;
      delay(20);
      
    }
    i = serv->current_pos;
    //return to idle position
    while (serv->state){
      if (serv->current_pos == (serv->idle_pos)-1){
        serv->state = 0;
      }
      (serv->connected_servo).write(i);
      serv->current_pos = i;
      i++;
      delay(10);
    }
  }
}

void move_servos_to_idle(struct servoMot *serv1, struct servoMot *serv2){
  //move to starting position
  (serv1->connected_servo).write(serv1->idle_pos);
  serv1->current_pos = serv1->idle_pos;
  (serv2->connected_servo).write(serv2->idle_pos);
  serv2->current_pos = serv2->idle_pos;
}

void initialize_servos(){
  //initialize servo structs
  //servo 1
  fetcher_struct.state = 0;
  fetcher_struct.current_pos = 0;
  //set idle and targetpos
  fetcher_struct.idle_pos = 80;
  fetcher_struct.target_pos = 158;
  fetcher_struct.connected_servo = servo_fetcher;
  //Servo2
  returner_struct.state = 0;
  returner_struct.current_pos = 0;
  //set idle and targetpos
  returner_struct.idle_pos = 120;
  returner_struct.target_pos = 156;
  returner_struct.connected_servo = servo_returner;

  //attach outputs to servo
  servo_fetcher.attach(5);
  servo_returner.attach(6);

  //move to starting position
  move_servos_to_idle(&fetcher_struct, &returner_struct);
}
//-------------------------------------------------------------------


//--------------------game methods-----------------------------------
void initialize_game(){
  game.plant_to_be_fetched = 0;
  game.torture_table = 0;
}
//-------------------------------------------------------------------


//interrupt of turntable switch, calibrates position
 void pin_ISR(){
  #ifdef DEBUGSTEP
  Serial.println("interrupt");
  #endif
  stepper1.current_pos = 0;
}
