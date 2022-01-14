# 1 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
/*

 * Author       :  Derek Zhou

 * Date         :  06/08/2021

 * Version      :  Generation 4 Version X Pattern 6

 * New Function :  Logo Test with new pin config

 */
# 8 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
# 9 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 2
# 10 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 2
# 11 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 2
# 12 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 2
# 13 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 2
# 14 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 2
# 15 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 2


# 16 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
void TaskMain(void *pvParameters);
void TaskStepper1(void *pvParameters);
void TaskStepper2(void *pvParameters);
void TaskStepper3(void *pvParameters);
void TaskStepper4(void *pvParameters);
# 31 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
// warning! 0 1 serial0, 2 interrupt, 9 10 Timer2, 44-46 unused PWM, 14 15 USART3
const uint8_t Pin_ButtonGo=22;
const uint8_t Pin_ButtonStart=23;
const uint8_t Pin_ButtonRegret=24;
const uint8_t Pin_LedGreen=25;
const uint8_t Pin_LedBlue=26;
const uint8_t Pin_LedRed=27;
const uint8_t Pin_Stepper1Enable=41;
const uint8_t Pin_Stepper1Pulse=42;
const uint8_t Pin_Stepper1Direction=43;
const uint8_t Pin_Stepper2Enable=41;
const uint8_t Pin_Stepper2Pulse=44;
const uint8_t Pin_Stepper2Direction=45;
const uint8_t Pin_Stepper3Enable=41;
const uint8_t Pin_Stepper3Pulse=46;
const uint8_t Pin_Stepper3Direction=47;
const uint8_t Pin_Stepper4Enable=41;
const uint8_t Pin_Stepper4Pulse=48;
const uint8_t Pin_Stepper4Direction=49;
const uint8_t Pin_Magnet1=A7;
const uint8_t Pin_Servo1=12;
const uint8_t Pin_Servo2=13;
const uint8_t Pin_Mp3Tx=50;
const uint8_t Pin_Mp3Rx=51;
const uint8_t Pin_Random=A0;

// task & queue handler
TaskControlBlock_t *TaskHandle_Stepper1, *TaskHandle_Stepper2;
QueueHandle_t QueueHandle_Angle1, QueueHandle_Angle2, QueueHandle_Finish1, QueueHandle_Finish2;
TaskControlBlock_t *TaskHandle_Stepper3, *TaskHandle_Stepper4;
QueueHandle_t QueueHandle_Angle3, QueueHandle_Angle4, QueueHandle_Finish3, QueueHandle_Finish4;
// init & start
bool BeginMainLoopFlag=false;
bool InitOkFlag=false;
bool FaceRecognitionFlag=false;
bool StartOkFlag=false;
bool GameStartFlag=false;
bool RobotWorkFlag=false;
// protocol
char globalinchar;
bool getkeywordflag=false;
String keyword="";
String coordinatestring="";
String result="";
String error="";
String faceinfo="";
bool stringcomplete=false;
uint8_t coordinatecount=0; // count the coordinate char
uint8_t coordinatedata[8]={0};
float currentposition[2]={0};
float targetposition[2]={0};
// stepper control
extern const float circle5mmangle1[127] 
# 83 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                                  __attribute__((__progmem__))
# 83 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                         ;
extern const float circle5mmangle2[127] 
# 84 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                                  __attribute__((__progmem__))
# 84 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                         ;
extern const float circle3mmangle1[211] 
# 85 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                                  __attribute__((__progmem__))
# 85 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                         ;
extern const float circle3mmangle2[211] 
# 86 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                                  __attribute__((__progmem__))
# 86 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                         ;
extern const float circle2mmangle1[316] 
# 87 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                                  __attribute__((__progmem__))
# 87 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                         ;
extern const float circle2mmangle2[316] 
# 88 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                                  __attribute__((__progmem__))
# 88 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                         ;
extern const float circle1mmangle1[630] 
# 89 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                                  __attribute__((__progmem__))
# 89 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                         ;
extern const float circle1mmangle2[630] 
# 90 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                                  __attribute__((__progmem__))
# 90 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                         ;
extern const float lineangle1[362 /*1mm*/] 
# 91 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                       __attribute__((__progmem__))
# 91 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                              ;
extern const float lineangle2[362 /*1mm*/] 
# 92 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                       __attribute__((__progmem__))
# 92 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                              ;
extern const float squareangle1[805 /*1mm*/] 
# 93 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                           __attribute__((__progmem__))
# 93 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                  ;
extern const float squareangle2[805 /*1mm*/] 
# 94 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                           __attribute__((__progmem__))
# 94 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                  ;
extern const float faceangle1[268 /*3mm*/] 
# 95 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                       __attribute__((__progmem__))
# 95 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                              ;
extern const float faceangle2[268 /*3mm*/] 
# 96 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                       __attribute__((__progmem__))
# 96 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                              ;
extern const float lefteyeangle1[42 /*3mm*/] 
# 97 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                             __attribute__((__progmem__))
# 97 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                    ;
extern const float lefteyeangle2[42 /*3mm*/] 
# 98 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                             __attribute__((__progmem__))
# 98 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                    ;
extern const float righteyeangle1[42 /*3mm*/] 
# 99 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                               __attribute__((__progmem__))
# 99 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                      ;
extern const float righteyeangle2[42 /*3mm*/] 
# 100 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                               __attribute__((__progmem__))
# 100 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                      ;
extern const float mouthangle1[60 /*3mm*/] 
# 101 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         __attribute__((__progmem__))
# 101 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                ;
extern const float mouthangle2[60 /*3mm*/] 
# 102 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         __attribute__((__progmem__))
# 102 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                ;
extern const float logoangle1[197 /*3mm*/] 
# 103 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                       __attribute__((__progmem__))
# 103 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                              ;
extern const float logoangle2[197 /*3mm*/] 
# 104 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                       __attribute__((__progmem__))
# 104 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                              ;
int pulsenumber1=0;
int pulsenumber2=0;
int pulsenumber3=0;
int pulsenumber4=0;
int pulsecnt1=0;
int pulsecnt2=0;
int pulsecnt3=0;
int pulsecnt4=0;
float missangle3=0;
float missangle4=0;
// MP3
SoftwareSerial Mp3Serial(Pin_Mp3Tx, Pin_Mp3Rx); //50--module TX, 51--module RX
// servo
Servo servo1, servo2;
const int Servo1MaxAngle=120;
const int Servo1MinAngle=75;
const int Servo2MaxAngle=110;
const int Servo2MinAngle=80;

void setup()
{
    SerialInit();
    LedInit();
    ButtonInit();
    StepperMotorInit();
    Mp3Init();
    MagnetInit();
    ServoInit();
    FlexiTimer2Init();
    RandomInit();

    QueueHandle_Angle1=xQueueGenericCreate( ( 1 ), ( sizeof(float) ), ( ( ( uint8_t ) 0U ) ) );
    QueueHandle_Angle2=xQueueGenericCreate( ( 1 ), ( sizeof(float) ), ( ( ( uint8_t ) 0U ) ) );
    QueueHandle_Angle3=xQueueGenericCreate( ( 1 ), ( sizeof(float) ), ( ( ( uint8_t ) 0U ) ) );
    QueueHandle_Angle4=xQueueGenericCreate( ( 1 ), ( sizeof(float) ), ( ( ( uint8_t ) 0U ) ) );
    QueueHandle_Finish1=xQueueGenericCreate( ( 1 ), ( sizeof(bool) ), ( ( ( uint8_t ) 0U ) ) );
    QueueHandle_Finish2=xQueueGenericCreate( ( 1 ), ( sizeof(bool) ), ( ( ( uint8_t ) 0U ) ) );
    QueueHandle_Finish3=xQueueGenericCreate( ( 1 ), ( sizeof(bool) ), ( ( ( uint8_t ) 0U ) ) );
    QueueHandle_Finish4=xQueueGenericCreate( ( 1 ), ( sizeof(bool) ), ( ( ( uint8_t ) 0U ) ) );
    Serial.println("Info queues have been created! ");

    xTaskCreate(TaskMain, "Main", 1024, 
# 146 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                       __null
# 146 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                           , 2, 
# 146 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                __null
# 146 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                    ); //can also be recognized as protocol+led+button task
    Serial.println("Tasks have been created! ");

    // Serial0PrintLastCoordinateArrayAddress();
    Serial.println("Robot has been initialized!\n");
}
void loop()
{
    ;
}

// several tasks that need to be managed by FreeRTOS
void TaskMain(void *pvParameters)
{
    (void) pvParameters;

    /*

    //initialization

    RobotHandshake("#initok\n"); //robot sends #initok and wait M20 sends #ok

    while(InitOkFlag==false) //robot wait M20 to send #initok and send #ok to answer

    {

        CheckInitOk();

    }



    //face recognition

    while(FaceRecognitionFlag==false)

    {

        CheckFaceRecognition();

    }

    */
# 177 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
    // robot is ready
    vTaskDelay(500/( (TickType_t) 
# 178 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                  (1 << (0 
# 178 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                  /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 178 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                  )) 
# 178 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                  ));
    // Mp3Execute(32);
    RobotIsReady();
    // Mp3Execute(31);
    vTaskDelay(500/( (TickType_t) 
# 182 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                  (1 << (0 
# 182 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                  /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 182 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                  )) 
# 182 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                  ));

    /*

    //start

    while(GameStartFlag==false) //robot wait the player to press the start button

    {

        ButtonStartScan(); //contains CheckStartOk();

    }

    */
# 192 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
    BeginMainLoopFlag=true;
    Serial.println("Get into the main loop! \n");

    //main loop
    while(1)
    {
        /*

        if(RobotWorkFlag==false) //counter disturbances of button while robot is working

        {

            ButtonGoScan();

            ButtonRegretScan();

        }



        while(Serial3.available() && stringcomplete==false) //jump out as long as "true"

        {

            globalinchar=(char)Serial3.read();

            Serial3ByteTranslate(globalinchar);

            vTaskDelay(10/portTICK_PERIOD_MS); //wait the input char to be totally received into the buffer

        }

        if(stringcomplete==true)

        {

            RobotWorkFlag=true;

            Serial0PrintRxData();

            SendTxOK(keyword);

            AnalyzeRxData(keyword);

            ClearOut();

            RobotWorkFlag=false;

        }



        vTaskDelay(10/portTICK_PERIOD_MS); // avoid task jam

        */
# 224 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
        RobotDrawLogo();

        vTaskDelay(5000/( (TickType_t) 
# 226 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                       (1 << (0 
# 226 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                       /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 226 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                       )) 
# 226 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                       ));
    }
}
void TaskStepper1(void *pvParameters)
{
    (void) pvParameters;

    float Stepper1Angle;
    bool Stepper1Finish=false;

    while(1)
    {
        if(xQueueReceive(QueueHandle_Angle1, &Stepper1Angle, ( TickType_t ) 0xffff))
        {
            // digitalWrite(Pin_Stepper1Enable, LOW);
            TaskStepper1_AngleToPulseAndRotate(Stepper1Angle);
            // digitalWrite(Pin_Stepper1Enable, HIGH);
        }
        Stepper1Finish=true;
        xQueueGenericSend( ( QueueHandle_Finish1 ), ( &Stepper1Finish ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
        Stepper1Angle=0;
        Stepper1Finish=false;
        vTaskDelay(10/( (TickType_t) 
# 248 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                     (1 << (0 
# 248 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                     /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 248 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                     )) 
# 248 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                     ));
    }
}
void TaskStepper2(void *pvParameters)
{
    (void) pvParameters;

    float Stepper2Angle;
    bool Stepper2Finish=false;

    while(1)
    {
        if(xQueueReceive(QueueHandle_Angle2, &Stepper2Angle, ( TickType_t ) 0xffff))
        {
            // digitalWrite(Pin_Stepper2Enable, LOW);
            TaskStepper2_AngleToPulseAndRotate(Stepper2Angle);
            // digitalWrite(Pin_Stepper2Enable, HIGH);
        }
        Stepper2Finish=true;
        xQueueGenericSend( ( QueueHandle_Finish2 ), ( &Stepper2Finish ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
        Stepper2Angle=0;
        Stepper2Finish=false;
        vTaskDelay(10/( (TickType_t) 
# 270 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                     (1 << (0 
# 270 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                     /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 270 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                     )) 
# 270 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                     ));
    }
}
void TaskStepper3(void *pvParameters)
{
    (void) pvParameters;

    float Stepper3Angle;
    bool Stepper3Finish=false;

    while(1)
    {
        if(xQueueReceive(QueueHandle_Angle3, &Stepper3Angle, ( TickType_t ) 0xffff))
        {
            TaskStepper3_AngleToPulseAndRotate(Stepper3Angle);
        }
        Stepper3Finish=true;
        xQueueGenericSend( ( QueueHandle_Finish3 ), ( &Stepper3Finish ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
        Stepper3Angle=0;
        Stepper3Finish=false;
        vTaskDelay(10/( (TickType_t) 
# 290 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                     (1 << (0 
# 290 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                     /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 290 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                     )) 
# 290 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                     ));
    }
}
void TaskStepper4(void *pvParameters)
{
    (void) pvParameters;

    float Stepper4Angle;
    bool Stepper4Finish=false;

    while(1)
    {
        if(xQueueReceive(QueueHandle_Angle4, &Stepper4Angle, ( TickType_t ) 0xffff))
        {
            TaskStepper4_AngleToPulseAndRotate(Stepper4Angle);
        }
        Stepper4Finish=true;
        xQueueGenericSend( ( QueueHandle_Finish4 ), ( &Stepper4Finish ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
        Stepper4Angle=0;
        Stepper4Finish=false;
        vTaskDelay(10/( (TickType_t) 
# 310 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                     (1 << (0 
# 310 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                     /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 310 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                     )) 
# 310 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                     ));
    }
}

// Serial 0 & 3
void SerialInit(void)
{
    Serial.begin(115200);
    Serial3.begin(115200);
    Serial.println("\nSerial module (0 & 3) has been initialized! ");
    delay(10);
}
// robot send command and wait for #ok & judge the handshake result
void RobotHandshake(String data)
{
    int retryCount = 3;
    int maxWaitTimeMs; //for defining only

    for(int i = 0; i < retryCount; i++)
    {
     //把之前的缓存清掉
  while (Serial3.available())
        {
            Serial3.read();
        }
  //发送数据
  Serial3.print(data);
  //发送完成后，等待"#ok\n”最长等待20ms
        maxWaitTimeMs = 20; //重新赋值，刷新for循环的操作
  while(Serial3.available()==0 && maxWaitTimeMs>0)
        {
   vTaskDelay(1/( (TickType_t) 
# 341 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
               (1 << (0 
# 341 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
               /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 341 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
               )) 
# 341 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
               ));
   maxWaitTimeMs--;
  }
  //确定有数据后，再等2ms，确保#ok全部收完
  if(maxWaitTimeMs==0)
        {
      vTaskDelay(2/( (TickType_t) 
# 347 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                  (1 << (0 
# 347 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                  /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 347 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                  )) 
# 347 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                  ));
      String okResp = Serial3.readStringUntil('\n');
            // Serial.print("okResp=");
            // Serial.println(okResp);
            Serial.print(">> ");

   //收到 ok 直接返回，否则重发
   if(okResp == "#ok\r" || okResp == "#ok")
            {
                HandShakeJudge(data, true);
                return; //successfully received ok from M20
   }
  }
 }
    HandShakeJudge(data, false);
}
void HandShakeJudge(String data, bool judgeflag)
{
    if(judgeflag==true)
    {
        Serial.println("Handshake successes, M20 sends #ok back to robot~ ");
    }
    else if(judgeflag==false)
    {
        Serial.println("Handshake fails... ");
        if(data=="#initok\n")
        {
            Serial.println("Robot-M20 initialization error! Please check the hardware serial connection and M20 startup! ");
            CheckErrorStatus(true);
            Mp3Execute(1);
        }
    }
}
// check stuff, fake protocol, disposable command
void CheckInitOk(void)
{
    while(Serial3.available() && stringcomplete==false) //jump out as long as "true"
    {
        globalinchar=(char)Serial3.read();
        Serial3ByteTranslate(globalinchar);
        vTaskDelay(10/( (TickType_t) 
# 387 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                     (1 << (0 
# 387 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                     /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 387 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                     )) 
# 387 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                     )); //wait the input char to be totally received into the buffer
    }
    if(stringcomplete==true)
    {
        Serial0PrintRxData();
        if(keyword=="initok" || keyword=="error")
        {
            SendTxOK(keyword);
            AnalyzeRxData(keyword);
            if(keyword=="initok")
            {
                InitOkFlag=true;
            }
        }
        else
        {
            Serial.println("Sorry, only receive #initok right now~ ");
        }
        ClearOut();
    }

    vTaskDelay(10/( (TickType_t) 
# 408 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 408 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 408 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 408 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
}
void CheckFaceRecognition(void)
{
    while(Serial3.available() && stringcomplete==false) //jump out as long as "true"
    {
        globalinchar=(char)Serial3.read();
        Serial3ByteTranslate(globalinchar);
        vTaskDelay(10/( (TickType_t) 
# 416 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                     (1 << (0 
# 416 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                     /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 416 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                     )) 
# 416 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                     )); //wait the input char to be totally received into the buffer
    }
    if(stringcomplete==true)
    {
        Serial0PrintRxData();
        if(keyword=="reo" || keyword=="error")
        {
            SendTxOK(keyword);
            AnalyzeRxData(keyword);
            if(faceinfo=="stranger" || faceinfo=="@lb" || faceinfo=="@cys" || faceinfo=="@zmy" || faceinfo=="@tmy" || faceinfo=="@sh" || faceinfo=="@xl")
            {
                FaceRecognitionFlag=true;
            }
        }
        else
        {
            Serial.println("Sorry, only receive #reo right now~ ");
        }
        ClearOut();
    }

    vTaskDelay(10/( (TickType_t) 
# 437 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 437 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 437 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 437 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
}
void CheckStartOk(void)
{
    while(Serial3.available() && stringcomplete==false) //jump out as long as "true"
    {
        globalinchar=(char)Serial3.read();
        Serial3ByteTranslate(globalinchar);
        vTaskDelay(10/( (TickType_t) 
# 445 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                     (1 << (0 
# 445 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                     /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 445 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                     )) 
# 445 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                     )); //wait the input char to be totally received into the buffer
    }
    if(stringcomplete==true)
    {
        Serial0PrintRxData();
        if(keyword=="startok"|| keyword=="error")
        {
            SendTxOK(keyword);
            AnalyzeRxData(keyword);
            if(keyword=="startok")
            {
                LedSwitch(Pin_LedGreen, 1);
                StartOkFlag=true;
            }
        }
        else
        {
            Serial.println("Sorry, only receive #startok right now~ ");
        }
        ClearOut();
    }

    vTaskDelay(10/( (TickType_t) 
# 467 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 467 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 467 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 467 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
}
// translate, printf, OK, analyze, disposable & main loop
void Serial3ByteTranslate(char inchar)
{
    if(inchar==35) // # keyword announcement begin
    {
        getkeywordflag=true;
    }
    else if(inchar==58) // : keyword announcement finish
    {
        getkeywordflag=false;
    }
    else if(inchar==10) // \ keyword announcement & command finish \:92 \n:10 command: #initfinish no\n!!!!
    {
        getkeywordflag=false;
        stringcomplete=true;
    }
    else if(getkeywordflag==true && (inchar>=97 && inchar<=122))
    {
        keyword+=inchar;
    }
    else if((keyword=="move" || keyword=="eat" || keyword=="movec" || keyword=="eatc" || keyword=="movew" ||keyword=="eatw")
                                                                    && isDigit(inchar) && coordinatecount<8) // get 4 data only
    {
        coordinatestring+=inchar;
        coordinatecount++;
    }
    else if(keyword=="result" && (inchar>=97 && inchar<=122))
    {
        result+=inchar;
    }
    else if(keyword=="error" && isDigit(inchar))
    {
        error+=inchar;
    }
    else if(keyword=="reo" && ((inchar>=97 && inchar<=122) || inchar==0x40)) //a-z @
    {
        faceinfo+=inchar;
    }
}
void Serial0PrintRxData(void)
{
    Serial.print("\n");
    Serial.println("********************************");

    Serial.print("The keyword is: ");
    Serial.println(keyword);

    if(keyword=="move" || keyword=="movec" || keyword=="movew" || keyword=="eat" || keyword=="eatc" || keyword=="eatw")
    {
        Serial.print("The coordinate string is: ");
        if(coordinatestring=="")
        {
            Serial.println("NULL");
        }
        else
        {
            Serial.println(coordinatestring);
        }
    }
    if(keyword=="result")
    {
        Serial.print("The result is: ");
        Serial.println(result);
    }
    if(keyword=="error")
    {
        Serial.print("The error is: ");
        Serial.println(error);
    }
    if(keyword=="reo")
    {
        Serial.print("The face-info-string is: ");
        Serial.println(faceinfo);
    }
}
void SendTxOK(String keyword)
{
    Serial.println("********************************");
    if(keyword=="initok" || keyword=="move" || keyword=="eat" || keyword=="movec" || keyword=="eatc" || keyword=="movew" ||
                        keyword=="eatw" || keyword=="result" || keyword=="warn" || keyword=="error" || keyword=="retraok" ||
                                                                keyword=="retraerror" || keyword=="startok" || keyword=="reo")
    {
        Serial.println("Send Tx OK to M20. ");
        Serial3.print("#ok\n");
    }
    else if(keyword=="ok")
    {
        // no work to down
        // Serial.println("M20 says OK for this command. ");
    }
}
void AnalyzeRxData(String keyword)
{
    Serial.println("********************************");
    Serial.println("Recognizing the keyword case by case... ");

    //fake protocol --disposable command
    if(BeginMainLoopFlag==false)
    {
        if(keyword=="initok")
        {
            Serial.println("M20 part has been initialized! Let's go! ");
        }
        else if(keyword=="reo")
        {
            Serial.println("Robot has gotten the answer~ ");
            AnnounceFaceInfo(faceinfo);
        }
        else if(keyword=="startok")
        {
            Serial.println("M20 says startok~ ");
        }
        else if(keyword=="error")
        {
            Serial.println("!!!!!!!!!!!! ERROR !!!!!!!!!!!!! ");
            AnnounceError(error);
        }
    }
    else //real protocol --in main loop
    {
        if(keyword=="ok")
        {
            Serial.println("M20 has confirmed that this step is OK, robot doesn't need to send data. ");
        }
        else if(keyword=="move")
        {
            Serial.println("MOVE! ");
            CoordinateDataToArray(keyword, coordinatestring); //analyze the coordinate data
            Serial0PrintMoveOrEat(keyword);
            Mp3Execute(3);
            RobotMove(); // RobotMoveGranularity(); //to memorize the dean's method
            Mp3Execute(4);
            LedSwitch(Pin_LedGreen, 1);
        }
        else if(keyword=="eat")
        {
            Serial.println("EAT! ");
            CoordinateDataToArray(keyword, coordinatestring);
            Serial0PrintMoveOrEat(keyword);
            Mp3Execute(3);
            RobotEat();
            Mp3Execute(4);
            LedSwitch(Pin_LedGreen, 1);
        }
        else if(keyword=="movec")
        {
            Serial.println("CHECK! -MOVE-");
            CoordinateDataToArray(keyword, coordinatestring);
            Serial0PrintMoveOrEat(keyword);
            Mp3Execute(3);
            RobotMove(); // RobotMoveGranularity(); //to memorize the dean's method
            Mp3Execute(5); //check
            Mp3Execute(4);
            LedSwitch(Pin_LedGreen, 1);
        }
        else if(keyword=="eatc")
        {
            Serial.println("CHECK! -EAT-");
            CoordinateDataToArray(keyword, coordinatestring);
            Serial0PrintMoveOrEat(keyword);
            Mp3Execute(3);
            RobotEat();
            Mp3Execute(5); //check
            Mp3Execute(4);
            LedSwitch(Pin_LedGreen, 1);
        }
        else if(keyword=="movew")
        {
            Serial.println("Checkmate! -MOVE- ");
            CoordinateDataToArray(keyword, coordinatestring);
            Serial0PrintMoveOrEat(keyword);
            Mp3Execute(3);
            RobotMove();
            AnnounceResult("win"); //robot win player lose
        }
        else if(keyword=="eatw")
        {
            Serial.println("Checkmate! -EAT- ");
            CoordinateDataToArray(keyword, coordinatestring);
            Serial0PrintMoveOrEat(keyword);
            Mp3Execute(3);
            RobotEat();
            AnnounceResult("win");
        }
        else if(keyword=="retraok")
        {
            Serial.println("Regret successfully~ "); // retraerror -> error
            Mp3Execute(9);
        }
        else if(keyword=="retraerror")
        {
            Serial.println("Regret failed... "); // retraerror -> error
            Mp3Execute(10);
        }
        else if(keyword=="result")
        {
            Serial.println("The result is... ");
            AnnounceResult(result);
        }
        else if(keyword=="warn")
        {
            Serial.println("!+++++++++! WARNING !++++++++++! ");
            Serial.println("Repeat the placement for 3 times, please change your strategy! ");
            Mp3Execute(21);
        }
        else if(keyword=="error")
        {
            Serial.println("!!!!!!!!!!!! ERROR !!!!!!!!!!!!! ");
            AnnounceError(error);
        }
        else
        {
            Serial.println("Robot cannot recognize the command or has already recognized disposable command. Please check! ");
        }
    }
}
// judge coordinate data with judgement & transform coordinate to angle
void CoordinateDataToArray(String keyword, String coordinatestring)
{
    //transform coordinate string to coordinate data & print one by one
    for(int i=0; i<coordinatecount; i++) //count<=7 is rational, which has been realized in the Serial3ByteTranslate()
    {
        coordinatedata[i]=int(coordinatestring[i]-48);
    }
    //create 2 coordinate arrays
    currentposition[0]=coordinatedata[0]+0.1*coordinatedata[1];
    currentposition[1]=coordinatedata[2]+0.1*coordinatedata[3];
    targetposition[0]=coordinatedata[4]+0.1*coordinatedata[5];
    targetposition[1]=coordinatedata[6]+0.1*coordinatedata[7];
}
void Serial0PrintMoveOrEat(String keyword)
{
    if(keyword=="move" || keyword=="movec" || keyword=="movew")
    {
        Serial.print("Robot is moving from (");
    }
    else if(keyword=="eat" || keyword=="eatc" || keyword=="eatw")
    {
        Serial.print("Robot is eating from (");
    }
    Serial.print(currentposition[0], 1);
    Serial.print(", ");
    Serial.print(currentposition[1], 1);
    Serial.print(")");
    Serial.print(" to (");
    Serial.print(targetposition[0], 1);
    Serial.print(", ");
    Serial.print(targetposition[1], 1);
    Serial.println(")... ");
}
float CoordinateArrayToAngle(int x, int y, uint8_t steppernumber)
{
    float stepper1angle, stepper2angle;

    // stepper1angle=pgm_read_float_near(&stepper1anglematrix[x][y]);
    // stepper2angle=pgm_read_float_near(&stepper2anglematrix[x][y]);

    stepper1angle=0;
    stepper2angle=0;

    if(steppernumber==1)
    {
        Serial.print("Stepper 1 angle: ");
        Serial.println(stepper1angle);
        return stepper1angle;
    }
    else if(steppernumber==2)
    {
        Serial.print("Stepper 2 angle: ");
        Serial.println(stepper2angle);
        return stepper2angle;
    }
}
void Serial0PrintLastCoordinateArrayAddress(void) //to check if the flash-reading progmem pointer is overflow
{
    // uint16_t a, b;
    // a=&stepper1anglematrix[80][90]; //(81,91)
    // b=&stepper2anglematrix[80][90];
    // Serial.println(a);
    // Serial.println(b);
    // Serial.print("\n");
}
// announce
void AnnounceResult(String result)
{
    if(result=="lose") //protocol is alternate, which indicates robot lose
    {
        Serial.println("Congrats! You WIN!!!! ");
        Mp3Execute(6);
        for(int i=0; i<10; i++)
        {
            LedSwitch(Pin_LedBlue, 1);
            vTaskDelay(200/( (TickType_t) 
# 761 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                          (1 << (0 
# 761 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                          /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 761 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                          )) 
# 761 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                          ));
            LedSwitch(Pin_LedBlue, 0);
            vTaskDelay(200/( (TickType_t) 
# 763 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                          (1 << (0 
# 763 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                          /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 763 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                          )) 
# 763 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                          ));
        }
        LedSwitch(Pin_LedGreen, 0);
        LedSwitch(Pin_LedBlue, 1);
    }
    else if(result=="win")
    {
        Serial.println("Sorry, you lose... ");
        Mp3Execute(7);
        LedSwitch(Pin_LedGreen, 0);
        LedSwitch(Pin_LedBlue, 1);
    }
    else if(result=="draw")
    {
        Serial.println("Oh, draw~ ");
        Mp3Execute(8);
        for(int i=0; i<5; i++)
        {
            LedSwitch(Pin_LedBlue, 1);
            vTaskDelay(200/( (TickType_t) 
# 782 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                          (1 << (0 
# 782 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                          /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 782 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                          )) 
# 782 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                          ));
            LedSwitch(Pin_LedBlue, 0);
            vTaskDelay(200/( (TickType_t) 
# 784 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                          (1 << (0 
# 784 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                          /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 784 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                          )) 
# 784 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                          ));
        }
        LedSwitch(Pin_LedGreen, 0);
        LedSwitch(Pin_LedBlue, 1);
    }
    else
    {
        Serial.println("Robot cannot recognize your result condition. ");
    }
}
void AnnounceError(String error)
{
    if(error=="70000")
    {
        Serial.println("Internet error, please check the network. ");
        Mp3Execute(11);
    }
    else if(error=="70001")
    {
        Serial.println("Server interal error, got the wrong answer. ");
        Mp3Execute(12);
    }
    else if(error=="70002")
    {
        Serial.println("Server ack response timeout, didn't get the response. ");
        Mp3Execute(12);
    }
    else if(error=="80100")
    {
        Serial.println("Opening chessman placement error, please check. ");
        Mp3Execute(13);
    }
    else if(error=="80200")
    {
        Serial.println("车 placement error! ");
        Mp3Execute(14);
    }
    else if(error=="80210")
    {
        Serial.println("马 placement error! ");
        Mp3Execute(15);
    }
    else if(error=="80220")
    {
        Serial.println("炮 placement error! ");
        Mp3Execute(16);
    }
    else if(error=="80230")
    {
        Serial.println("相 placement error! ");
        Mp3Execute(17);
    }
    else if(error=="80240")
    {
        Serial.println("士 placement error! ");
        Mp3Execute(18);
    }
    else if(error=="80250")
    {
        Serial.println("帅 placement error! ");
        Mp3Execute(19);
    }
    else if(error=="80260")
    {
        Serial.println("兵 placement error! ");
        Mp3Execute(20);
    }
    else if(error=="80270")
    {
        Serial.println("Multi-chessman placement error! ");
        Mp3Execute(22);
    }
    else
    {
        Serial.println("Robot cannot recognize your error condition. ");
    }
    CheckErrorStatus(true);
}
void CheckErrorStatus(bool errorflag)
{
    if(errorflag==true)
    {
        LedSwitch(Pin_LedRed, 1);
        vTaskDelay(500/( (TickType_t) 
# 867 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                      (1 << (0 
# 867 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                      /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 867 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                      )) 
# 867 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                      ));
        LedSwitch(Pin_LedRed, 0); // equals refresh in each loop
    }
    else
    {
        LedSwitch(Pin_LedRed, 0);
    }
}
void AnnounceFaceInfo(String faceinfo)
{
    if(faceinfo=="nobody")
    {
        Serial.println("No player has been recognized. ");
        Mp3Execute(23);
    }
    else if(faceinfo=="stranger")
    {
        Serial.println("Hi, strange player~ ");
        Mp3Execute(24);
    }
    else if(faceinfo=="@lb")
    {
        Serial.println("Hi, 鲁白~ ");
        Mp3Execute(26);
    }
    else if(faceinfo=="@cys")
    {
        Serial.println("Hi, 陈永胜~ ");
        Mp3Execute(25);
    }
    else if(faceinfo=="@zmy")
    {
        Serial.println("Hi, Derek Zhou 周茗岩~ ");
        Mp3Execute(27);
    }
    else if(faceinfo=="@tmy")
    {
        Serial.println("Hi, Tommy 唐明勇~ ");
        Mp3Execute(28);
    }
    else if(faceinfo=="@sh")
    {
        Serial.println("Hi, Dean 沈徽~ ");
        Mp3Execute(29);
    }
    else if(faceinfo=="@xl")
    {
        Serial.println("Hi, CEO 徐立~ ");
        Mp3Execute(30);
    }
    else //wrong status
    {
        Serial.println("Robot cannot recognize your face info ID. ");
    }
}
// clear out and get ready for the next Rx from M20
void ClearOut(void)
{
    Serial.println("********************************");
    Serial.println("The command has been executed. ");
    Serial.println("********************************");
    Serial.print("\n"); //clear the screen
    keyword="";
    coordinatestring="";
    coordinatecount=0;
    for(int j=0; j<coordinatecount; j++) //clear out
    {
        coordinatedata[j]=0;
    }
    result="";
    error="";
    faceinfo="";
    stringcomplete=false;
}

// LED
void LedInit(void)
{
    pinMode(Pin_LedGreen, 0x1);
    pinMode(Pin_LedBlue, 0x1);
    pinMode(Pin_LedRed, 0x1);

    for(int i=0; i<5; i++)
    {
        LedSwitch(Pin_LedGreen, 1);
        LedSwitch(Pin_LedBlue, 1);
        LedSwitch(Pin_LedRed, 1);
        delay(100);
        LedSwitch(Pin_LedGreen, 0);
        LedSwitch(Pin_LedBlue, 0);
        LedSwitch(Pin_LedRed, 0);
        delay(100);
    }
    LedSwitch(Pin_LedGreen, 0);
    LedSwitch(Pin_LedBlue, 1);
    LedSwitch(Pin_LedRed, 0);

    Serial.println("LED module has been initialized! ");
    delay(10);
}
void LedSwitch(uint8_t pin, bool status)
{
    if(status==1)
    {
        digitalWrite(pin, 0x1);
    }
    else if(status==0)
    {
        digitalWrite(pin, 0x0);
    }
}

// button
void ButtonInit(void)
{
    pinMode(Pin_ButtonGo, 0x0);
    pinMode(Pin_ButtonStart, 0x0);
    pinMode(Pin_ButtonRegret, 0x0);

    Serial.println("Button module has been initialized! ");
    delay(10);
}
void ButtonStartScan(void)
{
    int buttonstate=digitalRead(Pin_ButtonStart); //=0 equals button is down, which is rational to physical structure.
    static int buttonstartupflag=1;

    if(buttonstartupflag==1 && buttonstate==0)
    {
        delay(20); //debounce
        buttonstartupflag=0; //tag the button is down
        buttonstate=digitalRead(Pin_ButtonStart); //rejudge the button state!

        if(buttonstate==0) //if the button is down indeed
        {
            buttonstate=!buttonstate;
            Serial.println("The player has pressed the start button. Game start! ");
            for(int i=0; i<2; i++)
            {
                LedSwitch(Pin_LedBlue, 0);
                vTaskDelay(200/( (TickType_t) 
# 1007 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                              (1 << (0 
# 1007 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                              /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1007 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                              )) 
# 1007 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                              ));
                digitalWrite(Pin_LedBlue, 1);
                vTaskDelay(200/( (TickType_t) 
# 1009 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                              (1 << (0 
# 1009 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                              /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1009 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                              )) 
# 1009 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                              ));
            }
            LedSwitch(Pin_LedBlue, 0);
            RobotHandshake("#start\n");
            while(StartOkFlag==false)
            {
                CheckStartOk();
            }
            Mp3Execute(2);
            LedSwitch(Pin_LedGreen, 1);
            GameStartFlag=true;
        }
    }
    else if(buttonstate==1)
    {
        buttonstartupflag=1;
    }
}
void ButtonGoScan(void)
{
    int buttonstate=digitalRead(Pin_ButtonGo); //=0 equals button is down, which is rational to physical structure.
    static int buttongocount=0;
    static int buttongoupflag=1;

    if(buttongoupflag==1 && buttonstate==0)
    {
        delay(20); //debounce
        buttongoupflag=0; //tag the button is down
        buttonstate=digitalRead(Pin_ButtonGo); //rejudge the button state!

        if(buttonstate==0) //if the button is down indeed
        {
            buttonstate=!buttonstate;
            buttongocount++;
            RobotWorkFlag=true; //lock the button module
            Serial.println("The chess player has pressed the button. M20 needs to capture the image. ");
            Serial.print("Buttoncount is "); Serial.println(buttongocount);
            LedSwitch(Pin_LedGreen, 0);
            RobotHandshake("#cap\n"); //Serial3.print("#cap\n");
            MP3RandomGoExecute();
        }
    }
    else if(buttonstate==1)
    {
        buttongoupflag=1;
    }
}
void ButtonRegretScan(void)
{
    int buttonstate=digitalRead(Pin_ButtonRegret); //=0 equals button is down, which is rational to physical structure.
    static int buttonregretupflag=1;

    if(buttonregretupflag==1 && buttonstate==0)
    {
        delay(20); //debounce
        buttonregretupflag=0; //tag the button is down
        buttonstate=digitalRead(Pin_ButtonRegret); //rejudge the button state!

        if(buttonstate==0) //if the button is down indeed
        {
            buttonstate=!buttonstate;
            RobotWorkFlag=true; //lock the button module
            Serial.println("The player has pressed the regret button. Regret please! \n");
            for(int i=0; i<5; i++)
            {
                LedSwitch(Pin_LedGreen, 1);
                vTaskDelay(200/( (TickType_t) 
# 1075 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                              (1 << (0 
# 1075 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                              /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1075 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                              )) 
# 1075 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                              ));
                LedSwitch(Pin_LedGreen, 0);
                vTaskDelay(200/( (TickType_t) 
# 1077 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                              (1 << (0 
# 1077 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                              /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1077 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                              )) 
# 1077 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                              ));
            }
            RobotHandshake("#retra\n");
        }
    }
    else if(buttonstate==1)
    {
        buttonregretupflag=1;
    }
}

// FlexiTimer2 init for stepper control & its callback
void FlexiTimer2Init(void)
{
    FlexiTimer2::set(10, 1.0/10000, FlexiTimer2TogglePulse); // xtimes*0.00001sec T=basecnt*baseT 4 times fast
    FlexiTimer2::start(); //stepper rotating
    Serial.println("FlexiTimer2 module has been initialized! \n");
    delay(10);
}
void FlexiTimer2TogglePulse(void)
{
    static bool pulseflag1 = 0x1;
    static bool pulseflag2 = 0x1;
    static bool pulseflag3 = 0x1;
    static bool pulseflag4 = 0x1;

    //stepper1
    if(pulsecnt1<pulsenumber1)
    {
        digitalWrite(Pin_Stepper1Pulse, pulseflag1);
        pulseflag1 = !pulseflag1;
        pulsecnt1++;
    }
    else if(pulsecnt1==pulsenumber1 || pulsecnt1>pulsenumber1)
    {
        pulsecnt1=0;
        pulsenumber1=0;
    }
    //stepper2
    if(pulsecnt2<pulsenumber2)
    {
        digitalWrite(Pin_Stepper2Pulse, pulseflag2);
        pulseflag2 = !pulseflag2;
        pulsecnt2++;
    }
    else if(pulsecnt2==pulsenumber2 || pulsecnt2>pulsenumber2)
    {
        pulsecnt2=0;
        pulsenumber2=0;
    }
    //stepper3
    if(pulsecnt3<pulsenumber3)
    {
        digitalWrite(Pin_Stepper3Pulse, pulseflag3);
        pulseflag3 = !pulseflag3;
        pulsecnt3++;
    }
    else if(pulsecnt3==pulsenumber3 || pulsecnt3>pulsenumber3)
    {
        pulsecnt3=0;
        pulsenumber3=0;
    }
    //stepper4
    if(pulsecnt4<pulsenumber4)
    {
        digitalWrite(Pin_Stepper4Pulse, pulseflag4);
        pulseflag4 = !pulseflag4;
        pulsecnt4++;
    }
    else if(pulsecnt4==pulsenumber4 || pulsecnt4>pulsenumber4)
    {
        pulsecnt4=0;
        pulsenumber4=0;
    }
}

// stepper motor
void StepperMotorInit(void)
{
    pinMode(Pin_Stepper1Direction, 0x1);
    pinMode(Pin_Stepper1Pulse, 0x1);
    pinMode(Pin_Stepper1Enable, 0x1);
    pinMode(Pin_Stepper2Direction, 0x1);
    pinMode(Pin_Stepper2Pulse, 0x1);
    pinMode(Pin_Stepper2Enable, 0x1);
    pinMode(Pin_Stepper3Direction, 0x1);
    pinMode(Pin_Stepper3Pulse, 0x1);
    pinMode(Pin_Stepper3Enable, 0x1);
    pinMode(Pin_Stepper4Direction, 0x1);
    pinMode(Pin_Stepper4Pulse, 0x1);
    pinMode(Pin_Stepper4Enable, 0x1);

    digitalWrite(Pin_Stepper1Enable, 0x0);
    digitalWrite(Pin_Stepper2Enable, 0x0);
    digitalWrite(Pin_Stepper3Enable, 0x0);
    digitalWrite(Pin_Stepper4Enable, 0x0); //work
    // digitalWrite(Pin_Stepper1Enable, HIGH);
    // digitalWrite(Pin_Stepper2Enable, HIGH);
    // digitalWrite(Pin_Stepper3Enable, HIGH);
    // digitalWrite(Pin_Stepper4Enable, HIGH); //wait to work

    Serial.println("Stepper motor module has been initialized! ");
    delay(10);
}
// execute move & eat
void RobotMove(void)
{
    int x1=10*currentposition[0];
    int y1=10*currentposition[1];
    int x2=10*targetposition[0];
    int y2=10*targetposition[1];
    float stepper1angle1, stepper2angle1, stepper1angle2, stepper2angle2, delta1, delta2, return1, return2;
    bool stepper1finishflag=false;
    bool stepper2finishflag=false;

    // Serial.println(x1);
    // Serial.println(y1);
    // Serial.println(x2);
    // Serial.println(y2);

    servo1.attach(Pin_Servo1);
    vTaskDelay(100/( (TickType_t) 
# 1198 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                  (1 << (0 
# 1198 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                  /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1198 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                  )) 
# 1198 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                  ));

    //to current coordinate
    //1 create task
    stepper1angle1=3.6*CoordinateArrayToAngle(x1, y1, 1);
    stepper2angle1=3*CoordinateArrayToAngle(x1, y1, 2);
    xTaskCreate(TaskStepper1, "Stepper1Rotate", 128, 
# 1204 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                    __null
# 1204 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                        , 1, &TaskHandle_Stepper1);
    xTaskCreate(TaskStepper2, "Stepper2Rotate", 128, 
# 1205 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                    __null
# 1205 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                        , 1, &TaskHandle_Stepper2);
    vTaskDelay(10/( (TickType_t) 
# 1206 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1206 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1206 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1206 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    //2 transmit data
    xQueueGenericSend( ( QueueHandle_Angle1 ), ( &stepper1angle1 ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
    xQueueGenericSend( ( QueueHandle_Angle2 ), ( &stepper2angle1 ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
    vTaskDelay(10/( (TickType_t) 
# 1210 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1210 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1210 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1210 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    //3 receive data
    while(stepper1finishflag==false || stepper2finishflag==false)
    {
        xQueueReceive(QueueHandle_Finish1, &stepper1finishflag, ( TickType_t ) 0xffff);
        xQueueReceive(QueueHandle_Finish2, &stepper2finishflag, ( TickType_t ) 0xffff);
    }
    vTaskDelay(10/( (TickType_t) 
# 1217 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1217 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1217 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1217 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    //4 clear
    stepper1finishflag=false;
    stepper2finishflag=false;
    vTaskDelete(TaskHandle_Stepper1);
    vTaskDelete(TaskHandle_Stepper2);
    vTaskDelay(10/( (TickType_t) 
# 1223 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1223 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1223 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1223 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    Serial.println("...");

    //catch the chess
    vTaskDelay(500/( (TickType_t) 
# 1227 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 (1 << (0 
# 1227 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1227 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 )) 
# 1227 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 ));
    MagnetSwitch(1, 1);
    ServoRotate(1, 0);
    vTaskDelay(500/( (TickType_t) 
# 1230 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 (1 << (0 
# 1230 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1230 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 )) 
# 1230 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 ));
    ServoRotate(1, 1);
    vTaskDelay(500/( (TickType_t) 
# 1232 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 (1 << (0 
# 1232 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1232 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 )) 
# 1232 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 ));

    //to target coordinate
    stepper1angle2=3.6*CoordinateArrayToAngle(x2, y2, 1);
    stepper2angle2=3*CoordinateArrayToAngle(x2, y2, 2);
    delta1=stepper1angle2-stepper1angle1;
    delta2=stepper2angle2-stepper2angle1;
    vTaskDelay(10/( (TickType_t) 
# 1239 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1239 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1239 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1239 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    xTaskCreate(TaskStepper1, "Stepper1Rotate", 128, 
# 1240 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                    __null
# 1240 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                        , 1, &TaskHandle_Stepper1);
    xTaskCreate(TaskStepper2, "Stepper2Rotate", 128, 
# 1241 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                    __null
# 1241 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                        , 1, &TaskHandle_Stepper2);
    xQueueGenericSend( ( QueueHandle_Angle1 ), ( &delta1 ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
    xQueueGenericSend( ( QueueHandle_Angle2 ), ( &delta2 ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
    vTaskDelay(10/( (TickType_t) 
# 1244 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1244 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1244 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1244 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    while(stepper1finishflag==false || stepper2finishflag==false)
    {
        xQueueReceive(QueueHandle_Finish1, &stepper1finishflag, ( TickType_t ) 0xffff);
        xQueueReceive(QueueHandle_Finish2, &stepper2finishflag, ( TickType_t ) 0xffff);
    }
    vTaskDelay(10/( (TickType_t) 
# 1250 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1250 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1250 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1250 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    stepper1finishflag=false;
    stepper2finishflag=false;
    vTaskDelete(TaskHandle_Stepper1);
    vTaskDelete(TaskHandle_Stepper2);
    vTaskDelay(10/( (TickType_t) 
# 1255 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1255 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1255 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1255 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    Serial.println("...");

    //set the chess
    vTaskDelay(500/( (TickType_t) 
# 1259 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 (1 << (0 
# 1259 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1259 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 )) 
# 1259 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 ));
    ServoRotate(1, 0);
    MagnetSwitch(1, 0);
    vTaskDelay(500/( (TickType_t) 
# 1262 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 (1 << (0 
# 1262 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1262 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 )) 
# 1262 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 ));
    ServoRotate(1, 1);
    vTaskDelay(500/( (TickType_t) 
# 1264 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 (1 << (0 
# 1264 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1264 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 )) 
# 1264 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 ));

    //back to initial position
    return1=(-1)*stepper1angle2;
    return2=(-1)*stepper2angle2;
    xTaskCreate(TaskStepper1, "Stepper1Rotate", 128, 
# 1269 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                    __null
# 1269 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                        , 1, &TaskHandle_Stepper1);
    xTaskCreate(TaskStepper2, "Stepper2Rotate", 128, 
# 1270 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                    __null
# 1270 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                        , 1, &TaskHandle_Stepper2);
    vTaskDelay(10/( (TickType_t) 
# 1271 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1271 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1271 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1271 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    xQueueGenericSend( ( QueueHandle_Angle1 ), ( &return1 ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
    xQueueGenericSend( ( QueueHandle_Angle2 ), ( &return2 ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
    vTaskDelay(10/( (TickType_t) 
# 1274 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1274 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1274 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1274 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    while(stepper1finishflag==false || stepper2finishflag==false)
    {
        xQueueReceive(QueueHandle_Finish1, &stepper1finishflag, ( TickType_t ) 0xffff);
        xQueueReceive(QueueHandle_Finish2, &stepper2finishflag, ( TickType_t ) 0xffff);
    }
    vTaskDelay(10/( (TickType_t) 
# 1280 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1280 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1280 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1280 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    stepper1finishflag=false;
    stepper2finishflag=false;
    vTaskDelete(TaskHandle_Stepper1);
    vTaskDelete(TaskHandle_Stepper2);
    vTaskDelay(10/( (TickType_t) 
# 1285 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1285 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1285 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1285 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    Serial.println("Move step is done. ");

    servo1.detach();
    vTaskDelay(200/( (TickType_t) 
# 1289 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                  (1 << (0 
# 1289 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                  /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1289 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                  )) 
# 1289 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                  ));
}
void RobotEat(void)
{
    int x1=10*currentposition[0];
    int y1=10*currentposition[1];
    int x2=10*targetposition[0];
    int y2=10*targetposition[1];
    float stepper1angle1, stepper2angle1, stepper1angle2, stepper2angle2, delta1, delta2, return1, return2;
    bool stepper1finishflag=false;
    bool stepper2finishflag=false;

    servo1.attach(Pin_Servo1);
    vTaskDelay(100/( (TickType_t) 
# 1302 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                  (1 << (0 
# 1302 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                  /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1302 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                  )) 
# 1302 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                  ));

    //to target coordinate
    stepper1angle2=3.6*CoordinateArrayToAngle(x2,y2,1);
    stepper2angle2=3*CoordinateArrayToAngle(x2,y2,2);
    xTaskCreate(TaskStepper1, "Stepper1Rotate", 128, 
# 1307 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                    __null
# 1307 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                        , 1, &TaskHandle_Stepper1);
    xTaskCreate(TaskStepper2, "Stepper2Rotate", 128, 
# 1308 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                    __null
# 1308 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                        , 1, &TaskHandle_Stepper2);
    vTaskDelay(10/( (TickType_t) 
# 1309 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1309 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1309 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1309 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    xQueueGenericSend( ( QueueHandle_Angle1 ), ( &stepper1angle2 ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
    xQueueGenericSend( ( QueueHandle_Angle2 ), ( &stepper2angle2 ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
    vTaskDelay(10/( (TickType_t) 
# 1312 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1312 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1312 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1312 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    while(stepper1finishflag==false || stepper2finishflag==false)
    {
        xQueueReceive(QueueHandle_Finish1, &stepper1finishflag, ( TickType_t ) 0xffff);
        xQueueReceive(QueueHandle_Finish2, &stepper2finishflag, ( TickType_t ) 0xffff);
    }
    vTaskDelay(10/( (TickType_t) 
# 1318 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1318 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1318 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1318 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    stepper1finishflag=false;
    stepper2finishflag=false;
    vTaskDelete(TaskHandle_Stepper1);
    vTaskDelete(TaskHandle_Stepper2);
    vTaskDelay(10/( (TickType_t) 
# 1323 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1323 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1323 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1323 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    Serial.println("...");

    //catch the chess
    vTaskDelay(500/( (TickType_t) 
# 1327 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 (1 << (0 
# 1327 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1327 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 )) 
# 1327 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 ));
    MagnetSwitch(1, 1);
    ServoRotate(1, 0);
    vTaskDelay(500/( (TickType_t) 
# 1330 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 (1 << (0 
# 1330 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1330 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 )) 
# 1330 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 ));
    ServoRotate(1, 1);
    vTaskDelay(500/( (TickType_t) 
# 1332 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 (1 << (0 
# 1332 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1332 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 )) 
# 1332 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 ));

    //move the chess out (back to initial position)
    return1=(-1)*stepper1angle2;
    return2=(-1)*stepper2angle2;
    xTaskCreate(TaskStepper1, "Stepper1Rotate", 128, 
# 1337 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                    __null
# 1337 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                        , 1, &TaskHandle_Stepper1);
    xTaskCreate(TaskStepper2, "Stepper2Rotate", 128, 
# 1338 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                    __null
# 1338 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                        , 1, &TaskHandle_Stepper2);
    vTaskDelay(10/( (TickType_t) 
# 1339 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1339 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1339 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1339 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    xQueueGenericSend( ( QueueHandle_Angle1 ), ( &return1 ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
    xQueueGenericSend( ( QueueHandle_Angle2 ), ( &return2 ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
    vTaskDelay(10/( (TickType_t) 
# 1342 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1342 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1342 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1342 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    while(stepper1finishflag==false || stepper2finishflag==false)
    {
        xQueueReceive(QueueHandle_Finish1, &stepper1finishflag, ( TickType_t ) 0xffff);
        xQueueReceive(QueueHandle_Finish2, &stepper2finishflag, ( TickType_t ) 0xffff);
    }
    vTaskDelay(10/( (TickType_t) 
# 1348 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1348 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1348 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1348 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    stepper1finishflag=false;
    stepper2finishflag=false;
    vTaskDelete(TaskHandle_Stepper1);
    vTaskDelete(TaskHandle_Stepper2);
    vTaskDelay(10/( (TickType_t) 
# 1353 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1353 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1353 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1353 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    Serial.println("...");

    //drop the chess
    vTaskDelay(500*2/( (TickType_t) 
# 1357 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                   (1 << (0 
# 1357 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                   /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1357 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                   )) 
# 1357 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                   ));
    MagnetSwitch(1, 0);
    vTaskDelay(500*2/( (TickType_t) 
# 1359 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                   (1 << (0 
# 1359 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                   /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1359 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                   )) 
# 1359 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                   ));

    //to current coordinate
    stepper1angle1=3.6*CoordinateArrayToAngle(x1, y1, 1);
    stepper2angle1=3*CoordinateArrayToAngle(x1, y1, 2);
    xTaskCreate(TaskStepper1, "Stepper1Rotate", 128, 
# 1364 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                    __null
# 1364 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                        , 1, &TaskHandle_Stepper1);
    xTaskCreate(TaskStepper2, "Stepper2Rotate", 128, 
# 1365 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                    __null
# 1365 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                        , 1, &TaskHandle_Stepper2);
    vTaskDelay(10/( (TickType_t) 
# 1366 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1366 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1366 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1366 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    xQueueGenericSend( ( QueueHandle_Angle1 ), ( &stepper1angle1 ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
    xQueueGenericSend( ( QueueHandle_Angle2 ), ( &stepper2angle1 ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
    vTaskDelay(10/( (TickType_t) 
# 1369 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1369 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1369 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1369 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    while(stepper1finishflag==false || stepper2finishflag==false)
    {
        xQueueReceive(QueueHandle_Finish1, &stepper1finishflag, ( TickType_t ) 0xffff);
        xQueueReceive(QueueHandle_Finish2, &stepper2finishflag, ( TickType_t ) 0xffff);
    }
    vTaskDelay(10/( (TickType_t) 
# 1375 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1375 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1375 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1375 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    stepper1finishflag=false;
    stepper2finishflag=false;
    vTaskDelete(TaskHandle_Stepper1);
    vTaskDelete(TaskHandle_Stepper2);
    vTaskDelay(10/( (TickType_t) 
# 1380 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1380 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1380 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1380 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    Serial.println("...");

    //catch the chess
    vTaskDelay(500/( (TickType_t) 
# 1384 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 (1 << (0 
# 1384 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1384 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 )) 
# 1384 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 ));
    MagnetSwitch(1, 1);
    ServoRotate(1, 0);
    vTaskDelay(500/( (TickType_t) 
# 1387 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 (1 << (0 
# 1387 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1387 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 )) 
# 1387 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 ));
    ServoRotate(1, 1);
    vTaskDelay(500/( (TickType_t) 
# 1389 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 (1 << (0 
# 1389 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1389 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 )) 
# 1389 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 ));

    //to target coordinate
    stepper1angle2=3.6*CoordinateArrayToAngle(x2, y2, 1);
    stepper2angle2=3*CoordinateArrayToAngle(x2, y2, 2);
    delta1=stepper1angle2-stepper1angle1;
    delta2=stepper2angle2-stepper2angle1;
    xTaskCreate(TaskStepper1, "Stepper1Rotate", 128, 
# 1396 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                    __null
# 1396 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                        , 1, &TaskHandle_Stepper1);
    xTaskCreate(TaskStepper2, "Stepper2Rotate", 128, 
# 1397 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                    __null
# 1397 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                        , 1, &TaskHandle_Stepper2);
    vTaskDelay(10/( (TickType_t) 
# 1398 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1398 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1398 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1398 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    xQueueGenericSend( ( QueueHandle_Angle1 ), ( &delta1 ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
    xQueueGenericSend( ( QueueHandle_Angle2 ), ( &delta2 ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
    vTaskDelay(10/( (TickType_t) 
# 1401 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1401 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1401 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1401 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    while(stepper1finishflag==false || stepper2finishflag==false)
    {
        xQueueReceive(QueueHandle_Finish1, &stepper1finishflag, ( TickType_t ) 0xffff);
        xQueueReceive(QueueHandle_Finish2, &stepper2finishflag, ( TickType_t ) 0xffff);
    }
    vTaskDelay(10/( (TickType_t) 
# 1407 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1407 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1407 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1407 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    stepper1finishflag=false;
    stepper2finishflag=false;
    vTaskDelete(TaskHandle_Stepper1);
    vTaskDelete(TaskHandle_Stepper2);
    vTaskDelay(10/( (TickType_t) 
# 1412 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1412 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1412 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1412 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    Serial.println("...");

    //set the chess
    vTaskDelay(500/( (TickType_t) 
# 1416 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 (1 << (0 
# 1416 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1416 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 )) 
# 1416 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 ));
    ServoRotate(1, 0);
    MagnetSwitch(1, 0);
    vTaskDelay(500/( (TickType_t) 
# 1419 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 (1 << (0 
# 1419 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1419 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 )) 
# 1419 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 ));
    ServoRotate(1, 1);
    vTaskDelay(500/( (TickType_t) 
# 1421 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 (1 << (0 
# 1421 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1421 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 )) 
# 1421 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 ));

    //back to initial position
    return1=(-1)*stepper1angle2;
    return2=(-1)*stepper2angle2;
    xTaskCreate(TaskStepper1, "Stepper1Rotate", 128, 
# 1426 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                    __null
# 1426 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                        , 1, &TaskHandle_Stepper1);
    xTaskCreate(TaskStepper2, "Stepper2Rotate", 128, 
# 1427 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                    __null
# 1427 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                        , 1, &TaskHandle_Stepper2);
    vTaskDelay(10/( (TickType_t) 
# 1428 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1428 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1428 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1428 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    xQueueGenericSend( ( QueueHandle_Angle1 ), ( &return1 ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
    xQueueGenericSend( ( QueueHandle_Angle2 ), ( &return2 ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
    vTaskDelay(10/( (TickType_t) 
# 1431 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1431 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1431 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1431 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    while(stepper1finishflag==false || stepper2finishflag==false)
    {
        xQueueReceive(QueueHandle_Finish1, &stepper1finishflag, ( TickType_t ) 0xffff);
        xQueueReceive(QueueHandle_Finish2, &stepper2finishflag, ( TickType_t ) 0xffff);
    }
    vTaskDelay(10/( (TickType_t) 
# 1437 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1437 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1437 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1437 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    stepper1finishflag=false;
    stepper2finishflag=false;
    vTaskDelete(TaskHandle_Stepper1);
    vTaskDelete(TaskHandle_Stepper2);
    vTaskDelay(10/( (TickType_t) 
# 1442 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1442 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1442 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1442 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    Serial.println("Eat step is done. ");

    servo1.detach();
    vTaskDelay(200/( (TickType_t) 
# 1446 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                  (1 << (0 
# 1446 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                  /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1446 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                  )) 
# 1446 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                  ));
}
void RobotIsReady(void)
{
    float stepper1angle1, stepper2angle1, stepper3angle1, stepper4angle1;
    bool stepper1finishflag=false;
    bool stepper2finishflag=false;
    bool stepper3finishflag=false;
    bool stepper4finishflag=false;

    //1 create task
    stepper1angle1=3.6*90;
    stepper2angle1=3*90;
    stepper3angle1=3.6*90;
    stepper4angle1=3*(-90);
    xTaskCreate(TaskStepper1, "Stepper1Rotate", 128, 
# 1461 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                    __null
# 1461 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                        , 1, &TaskHandle_Stepper1);
    xTaskCreate(TaskStepper2, "Stepper2Rotate", 128, 
# 1462 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                    __null
# 1462 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                        , 1, &TaskHandle_Stepper2);
    xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, 
# 1463 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                    __null
# 1463 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                        , 1, &TaskHandle_Stepper3);
    xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, 
# 1464 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                    __null
# 1464 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                        , 1, &TaskHandle_Stepper4);
    vTaskDelay(10/( (TickType_t) 
# 1465 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1465 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1465 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1465 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    //2 transmit data
    xQueueGenericSend( ( QueueHandle_Angle1 ), ( &stepper1angle1 ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
    xQueueGenericSend( ( QueueHandle_Angle2 ), ( &stepper2angle1 ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
    xQueueGenericSend( ( QueueHandle_Angle3 ), ( &stepper3angle1 ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
    xQueueGenericSend( ( QueueHandle_Angle4 ), ( &stepper4angle1 ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
    vTaskDelay(10/( (TickType_t) 
# 1471 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1471 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1471 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1471 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    //3 receive data
    while(stepper1finishflag==false || stepper2finishflag==false || stepper3finishflag==false || stepper4finishflag==false)
    {
        xQueueReceive(QueueHandle_Finish1, &stepper1finishflag, ( TickType_t ) 0xffff);
        xQueueReceive(QueueHandle_Finish2, &stepper2finishflag, ( TickType_t ) 0xffff);
        xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, ( TickType_t ) 0xffff);
        xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, ( TickType_t ) 0xffff);
    }
    vTaskDelay(10/( (TickType_t) 
# 1480 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1480 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1480 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1480 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
    //4 clear
    stepper1finishflag=false;
    stepper2finishflag=false;
    stepper3finishflag=false;
    stepper4finishflag=false;
    vTaskDelete(TaskHandle_Stepper1);
    vTaskDelete(TaskHandle_Stepper2);
    vTaskDelete(TaskHandle_Stepper3);
    vTaskDelete(TaskHandle_Stepper4);
    vTaskDelay(10/( (TickType_t) 
# 1490 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 (1 << (0 
# 1490 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1490 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                 )) 
# 1490 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                 ));
}
// transform the angle to pulse  --Dividing the function for each stepper in order to transplant the FreeRTOS
void TaskStepper1_AngleToPulseAndRotate(float angle)
{
    //judge the direction
    if(angle>=0)
    {
        digitalWrite(Pin_Stepper1Direction, 0x1); //anti-clockwise! 
    }
    else
    {
        digitalWrite(Pin_Stepper1Direction, 0x0);
    }

    //calculate the pulse number
    pulsenumber1=2*((angle)>0?(angle):-(angle))*1600 /*θ=1.8°, microstep=8, ∴360°÷θ×microstep=1600*//360.00; //toggle!
    Serial.print("Stepper 1 pulse: ");
    Serial.println(pulsenumber1);
    // Serial.println("..."); //only stepper2 need ...!

    //execute the rotation
    while(pulsecnt1<pulsenumber1) //toggle!
    {
        vTaskDelay(20/( (TickType_t) 
# 1514 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                     (1 << (0 
# 1514 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                     /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1514 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                     )) 
# 1514 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                     ));
    }
    pulsecnt1=0;
    pulsenumber1=0;
}
void TaskStepper2_AngleToPulseAndRotate(float angle)
{
    //judge the direction
    if(angle>=0)
    {
        digitalWrite(Pin_Stepper2Direction, 0x0); //anti-clockwise! opposite!!!!
    }
    else
    {
        digitalWrite(Pin_Stepper2Direction, 0x1);
    }

    //calculate the pulse number
    pulsenumber2=2*((angle)>0?(angle):-(angle))*1600 /*θ=1.8°, microstep=8, ∴360°÷θ×microstep=1600*//360.00; //toggle!
    Serial.print("Stepper 2 pulse: ");
    Serial.println(pulsenumber2);

    //execute the rotation
    while(pulsecnt2<pulsenumber2)
    {
        vTaskDelay(20/( (TickType_t) 
# 1539 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                     (1 << (0 
# 1539 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                     /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1539 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                     )) 
# 1539 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                     ));
    }
    pulsecnt2=0;
    pulsenumber2=0;
}
void TaskStepper3_AngleToPulseAndRotate(float angle)
{
    float realangle3;

    //judge the direction
    if(angle>=0)
    {
        digitalWrite(Pin_Stepper3Direction, 0x0); //anti-clockwise! opposite!!!!
        angle+=missangle3;
    }
    else
    {
        digitalWrite(Pin_Stepper3Direction, 0x1);
        angle-=missangle3;
    }

    //calculate the pulse number
    pulsenumber3=2*((angle)>0?(angle):-(angle))*1600 /*θ=1.8°, microstep=8, ∴360°÷θ×microstep=1600*//360.00; //toggle!
    realangle3=pulsenumber3/2*360.00/1600 /*θ=1.8°, microstep=8, ∴360°÷θ×microstep=1600*/;
    missangle3=((angle)>0?(angle):-(angle))-((realangle3)>0?(realangle3):-(realangle3));

    //execute the rotation
    while(pulsecnt3<pulsenumber3)
    {
        vTaskDelay(20/( (TickType_t) 
# 1568 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                     (1 << (0 
# 1568 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                     /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1568 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                     )) 
# 1568 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                     ));
    }
    pulsecnt3=0;
    pulsenumber3=0;
}
void TaskStepper4_AngleToPulseAndRotate(float angle)
{
    float realangle4;

    //judge the direction
    if(angle>=0)
    {
        digitalWrite(Pin_Stepper4Direction, 0x0); //anti-clockwise! opposite!!!!
        angle+=missangle4;
    }
    else
    {
        digitalWrite(Pin_Stepper4Direction, 0x1);
        angle-=missangle4;
    }

    //calculate the pulse number
    pulsenumber4=2*((angle)>0?(angle):-(angle))*1600 /*θ=1.8°, microstep=8, ∴360°÷θ×microstep=1600*//360.00; //toggle!
    realangle4=pulsenumber4/2*360.00/1600 /*θ=1.8°, microstep=8, ∴360°÷θ×microstep=1600*/;
    missangle4=((angle)>0?(angle):-(angle))-((realangle4)>0?(realangle4):-(realangle4));
    // Serial.print("pulsenumber4="); Serial.println(pulsenumber4);
    // Serial.print("realangle4="); Serial.println(realangle4);
    // Serial.print("missangle4="); Serial.println(missangle4);

    //execute the rotation
    while(pulsecnt4<pulsenumber4)
    {
        vTaskDelay(20/( (TickType_t) 
# 1600 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                     (1 << (0 
# 1600 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                     /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1600 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                     )) 
# 1600 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                     ));
    }
    pulsecnt4=0;
    pulsenumber4=0;
}
// drawing command
void RobotDrawCircle5mm(void)
{
    float stepper3angle, stepper4angle;
    bool stepper3finishflag=false;
    bool stepper4finishflag=false;
    int i;
    float a,b;

    vTaskDelay(1000/( (TickType_t) 
# 1614 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                   (1 << (0 
# 1614 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                   /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1614 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                   )) 
# 1614 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                   ));

    for(i=0; i<127; i++)
    {
        a=
# 1618 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 1618 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &circle5mmangle1[i]
# 1618 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 1618 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                 ;
        b=
# 1619 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 1619 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &circle5mmangle2[i]
# 1619 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 1619 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                 ;
        stepper3angle=3.6*a*(-1);
        stepper4angle=3*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, 
# 1624 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 1624 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, 
# 1625 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 1625 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueGenericSend( ( QueueHandle_Angle3 ), ( &stepper3angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
        xQueueGenericSend( ( QueueHandle_Angle4 ), ( &stepper4angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, ( TickType_t ) 0xffff);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, ( TickType_t ) 0xffff);
        }

        //4 clear
        stepper3finishflag=false;
        stepper4finishflag=false;
        vTaskDelete(TaskHandle_Stepper3);
        vTaskDelete(TaskHandle_Stepper4);

        if(i==0)
        {
            servo2.attach(Pin_Servo2);
            vTaskDelay(500/( (TickType_t) 
# 1647 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         (1 << (0 
# 1647 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1647 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         )) 
# 1647 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         ));
            ServoRotate(2, 0);
            vTaskDelay(500/( (TickType_t) 
# 1649 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         (1 << (0 
# 1649 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1649 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         )) 
# 1649 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         ));
            Serial.println("Pen down!");
        }

        if(i==127 -1-1)
        {
            servo2.attach(Pin_Servo2);
            vTaskDelay(500/( (TickType_t) 
# 1656 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         (1 << (0 
# 1656 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1656 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         )) 
# 1656 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         ));
            ServoRotate(2, 1);
            vTaskDelay(500/( (TickType_t) 
# 1658 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         (1 << (0 
# 1658 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1658 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         )) 
# 1658 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         ));
            Serial.println("Pen up!\n");
        }

        // vTaskDelay(20/portTICK_PERIOD_MS);
    }
}
void RobotDrawCircle3mm(void)
{
    float stepper3angle, stepper4angle;
    bool stepper3finishflag=false;
    bool stepper4finishflag=false;
    int i;
    float a,b;

    vTaskDelay(1000/( (TickType_t) 
# 1673 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                   (1 << (0 
# 1673 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                   /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1673 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                   )) 
# 1673 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                   ));

    for(i=0; i<211; i++)
    {
        a=
# 1677 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 1677 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &circle3mmangle1[i]
# 1677 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 1677 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                 ;
        b=
# 1678 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 1678 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &circle3mmangle2[i]
# 1678 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 1678 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                 ;
        stepper3angle=3.6*a*(-1);
        stepper4angle=3*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, 
# 1683 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 1683 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, 
# 1684 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 1684 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueGenericSend( ( QueueHandle_Angle3 ), ( &stepper3angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
        xQueueGenericSend( ( QueueHandle_Angle4 ), ( &stepper4angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, ( TickType_t ) 0xffff);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, ( TickType_t ) 0xffff);
        }

        //4 clear
        stepper3finishflag=false;
        stepper4finishflag=false;
        vTaskDelete(TaskHandle_Stepper3);
        vTaskDelete(TaskHandle_Stepper4);

        if(i==0)
        {
            servo2.attach(Pin_Servo2);
            vTaskDelay(500/( (TickType_t) 
# 1706 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         (1 << (0 
# 1706 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1706 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         )) 
# 1706 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         ));
            ServoRotate(2, 0);
            vTaskDelay(500/( (TickType_t) 
# 1708 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         (1 << (0 
# 1708 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1708 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         )) 
# 1708 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         ));
            Serial.println("Pen down!");
        }

        if(i==211 -1-1)
        {
            servo2.attach(Pin_Servo2);
            vTaskDelay(500/( (TickType_t) 
# 1715 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         (1 << (0 
# 1715 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1715 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         )) 
# 1715 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         ));
            ServoRotate(2, 1);
            vTaskDelay(500/( (TickType_t) 
# 1717 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         (1 << (0 
# 1717 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1717 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         )) 
# 1717 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         ));
            Serial.println("Pen up!\n");
        }

        // vTaskDelay(20/portTICK_PERIOD_MS);
    }
}
void RobotDrawCircle2mm(void)
{
    float stepper3angle, stepper4angle;
    bool stepper3finishflag=false;
    bool stepper4finishflag=false;
    int i;
    float a,b;

    vTaskDelay(1000/( (TickType_t) 
# 1732 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                   (1 << (0 
# 1732 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                   /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1732 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                   )) 
# 1732 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                   ));

    for(i=0; i<316; i++)
    {
        a=
# 1736 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 1736 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &circle2mmangle1[i]
# 1736 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 1736 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                 ;
        b=
# 1737 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 1737 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &circle2mmangle2[i]
# 1737 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 1737 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                 ;
        stepper3angle=3.6*a*(-1);
        stepper4angle=3*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, 
# 1742 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 1742 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, 
# 1743 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 1743 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueGenericSend( ( QueueHandle_Angle3 ), ( &stepper3angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
        xQueueGenericSend( ( QueueHandle_Angle4 ), ( &stepper4angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, ( TickType_t ) 0xffff);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, ( TickType_t ) 0xffff);
        }

        //4 clear
        stepper3finishflag=false;
        stepper4finishflag=false;
        vTaskDelete(TaskHandle_Stepper3);
        vTaskDelete(TaskHandle_Stepper4);

        if(i==0)
        {
            servo2.attach(Pin_Servo2);
            vTaskDelay(500/( (TickType_t) 
# 1765 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         (1 << (0 
# 1765 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1765 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         )) 
# 1765 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         ));
            ServoRotate(2, 0);
            vTaskDelay(500/( (TickType_t) 
# 1767 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         (1 << (0 
# 1767 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1767 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         )) 
# 1767 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         ));
            Serial.println("Pen down!");
        }

        if(i==316 -1-1)
        {
            servo2.attach(Pin_Servo2);
            vTaskDelay(500/( (TickType_t) 
# 1774 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         (1 << (0 
# 1774 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1774 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         )) 
# 1774 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         ));
            ServoRotate(2, 1);
            vTaskDelay(500/( (TickType_t) 
# 1776 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         (1 << (0 
# 1776 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1776 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         )) 
# 1776 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         ));
            Serial.println("Pen up!\n");
        }

        // vTaskDelay(20/portTICK_PERIOD_MS);
    }
}
void RobotDrawCircle1mm(void)
{
    float stepper3angle, stepper4angle;
    bool stepper3finishflag=false;
    bool stepper4finishflag=false;
    int i;
    float a,b;

    vTaskDelay(1000/( (TickType_t) 
# 1791 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                   (1 << (0 
# 1791 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                   /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1791 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                   )) 
# 1791 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                   ));

    for(i=0; i<630; i++)
    {
        a=
# 1795 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 1795 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &circle1mmangle1[i]
# 1795 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 1795 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                 ;
        b=
# 1796 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 1796 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &circle1mmangle2[i]
# 1796 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 1796 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                 ;
        stepper3angle=3.6*a*(-1);
        stepper4angle=3*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, 
# 1801 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 1801 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, 
# 1802 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 1802 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueGenericSend( ( QueueHandle_Angle3 ), ( &stepper3angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
        xQueueGenericSend( ( QueueHandle_Angle4 ), ( &stepper4angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, ( TickType_t ) 0xffff);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, ( TickType_t ) 0xffff);
        }

        //4 clear
        stepper3finishflag=false;
        stepper4finishflag=false;
        vTaskDelete(TaskHandle_Stepper3);
        vTaskDelete(TaskHandle_Stepper4);

        if(i==0)
        {
            servo2.attach(Pin_Servo2);
            vTaskDelay(500/( (TickType_t) 
# 1824 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         (1 << (0 
# 1824 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1824 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         )) 
# 1824 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         ));
            ServoRotate(2, 0);
            vTaskDelay(500/( (TickType_t) 
# 1826 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         (1 << (0 
# 1826 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1826 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         )) 
# 1826 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         ));
            Serial.println("Pen down!");
        }

        if(i==630 -1-1)
        {
            servo2.attach(Pin_Servo2);
            vTaskDelay(500/( (TickType_t) 
# 1833 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         (1 << (0 
# 1833 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1833 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         )) 
# 1833 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         ));
            ServoRotate(2, 1);
            vTaskDelay(500/( (TickType_t) 
# 1835 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         (1 << (0 
# 1835 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1835 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         )) 
# 1835 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         ));
            Serial.println("Pen up!\n");
        }

        // vTaskDelay(20/portTICK_PERIOD_MS);
    }
}
void RobotDrawLine(void)
{
    float stepper3angle, stepper4angle;
    bool stepper3finishflag=false;
    bool stepper4finishflag=false;
    int i;
    float a,b;

    vTaskDelay(1000/( (TickType_t) 
# 1850 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                   (1 << (0 
# 1850 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                   /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1850 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                   )) 
# 1850 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                   ));

    for(i=0; i<362 /*1mm*/; i++)
    {
        a=
# 1854 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 1854 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &lineangle1[i]
# 1854 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 1854 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                            ;
        b=
# 1855 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 1855 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &lineangle2[i]
# 1855 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 1855 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                            ;
        stepper3angle=3.6*a*(-1);
        stepper4angle=3*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, 
# 1860 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 1860 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, 
# 1861 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 1861 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueGenericSend( ( QueueHandle_Angle3 ), ( &stepper3angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
        xQueueGenericSend( ( QueueHandle_Angle4 ), ( &stepper4angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, ( TickType_t ) 0xffff);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, ( TickType_t ) 0xffff);
        }

        //4 clear
        stepper3finishflag=false;
        stepper4finishflag=false;
        vTaskDelete(TaskHandle_Stepper3);
        vTaskDelete(TaskHandle_Stepper4);

        if(i==0)
        {
            servo2.attach(Pin_Servo2);
            vTaskDelay(500/( (TickType_t) 
# 1883 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         (1 << (0 
# 1883 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1883 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         )) 
# 1883 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         ));
            ServoRotate(2, 0);
            vTaskDelay(500/( (TickType_t) 
# 1885 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         (1 << (0 
# 1885 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1885 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         )) 
# 1885 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         ));
            Serial.println("Pen down!");
        }

        if(i==362 /*1mm*/-1-1)
        {
            servo2.attach(Pin_Servo2);
            vTaskDelay(500/( (TickType_t) 
# 1892 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         (1 << (0 
# 1892 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1892 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         )) 
# 1892 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         ));
            ServoRotate(2, 1);
            vTaskDelay(500/( (TickType_t) 
# 1894 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         (1 << (0 
# 1894 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1894 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         )) 
# 1894 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         ));
            Serial.println("Pen up!\n");
        }

        // vTaskDelay(20/portTICK_PERIOD_MS);
    }
}
void RobotDrawSquare(void)
{
    float stepper3angle, stepper4angle;
    bool stepper3finishflag=false;
    bool stepper4finishflag=false;
    int i;
    float a,b;

    vTaskDelay(1000/( (TickType_t) 
# 1909 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                   (1 << (0 
# 1909 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                   /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1909 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                   )) 
# 1909 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                   ));

    for(i=0; i<805 /*1mm*/; i++)
    {
        a=
# 1913 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 1913 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &squareangle1[i]
# 1913 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 1913 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                              ;
        b=
# 1914 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 1914 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &squareangle2[i]
# 1914 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 1914 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                              ;
        stepper3angle=3.6*a*(-1);
        stepper4angle=3*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, 
# 1919 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 1919 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, 
# 1920 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 1920 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueGenericSend( ( QueueHandle_Angle3 ), ( &stepper3angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
        xQueueGenericSend( ( QueueHandle_Angle4 ), ( &stepper4angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, ( TickType_t ) 0xffff);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, ( TickType_t ) 0xffff);
        }

        //4 clear
        stepper3finishflag=false;
        stepper4finishflag=false;
        vTaskDelete(TaskHandle_Stepper3);
        vTaskDelete(TaskHandle_Stepper4);

        if(i==0)
        {
            servo2.attach(Pin_Servo2);
            vTaskDelay(500/( (TickType_t) 
# 1942 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         (1 << (0 
# 1942 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1942 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         )) 
# 1942 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         ));
            ServoRotate(2, 0);
            vTaskDelay(500/( (TickType_t) 
# 1944 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         (1 << (0 
# 1944 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1944 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         )) 
# 1944 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         ));
            Serial.println("Pen down!");
        }

        if(i==805 /*1mm*/-1-1)
        {
            servo2.attach(Pin_Servo2);
            vTaskDelay(500/( (TickType_t) 
# 1951 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         (1 << (0 
# 1951 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1951 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         )) 
# 1951 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         ));
            ServoRotate(2, 1);
            vTaskDelay(500/( (TickType_t) 
# 1953 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         (1 << (0 
# 1953 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1953 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         )) 
# 1953 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         ));
            Serial.println("Pen up!\n");
        }

        // vTaskDelay(20/portTICK_PERIOD_MS);
    }
}
void RobotDrawDanbo(void)
{
    float stepper3angle, stepper4angle;
    bool stepper3finishflag=false;
    bool stepper4finishflag=false;
    int i;
    float a,b;

    vTaskDelay(1000/( (TickType_t) 
# 1968 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                   (1 << (0 
# 1968 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                   /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 1968 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                   )) 
# 1968 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                   ));

    //move to the first point
    for(i=0; i<1; i++)
    {
        a=
# 1973 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 1973 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &faceangle1[i]
# 1973 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 1973 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                            ;
        b=
# 1974 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 1974 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &faceangle2[i]
# 1974 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 1974 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                            ;
        stepper3angle=3.6*a*(-1);
        stepper4angle=3*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, 
# 1979 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 1979 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, 
# 1980 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 1980 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueGenericSend( ( QueueHandle_Angle3 ), ( &stepper3angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
        xQueueGenericSend( ( QueueHandle_Angle4 ), ( &stepper4angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, ( TickType_t ) 0xffff);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, ( TickType_t ) 0xffff);
        }

        //4 clear
        stepper3finishflag=false;
        stepper4finishflag=false;
        vTaskDelete(TaskHandle_Stepper3);
        vTaskDelete(TaskHandle_Stepper4);
    }
    PenDown();
    //draw face
    for(i=1; i<268 /*3mm*/; i++)
    {
        a=
# 2003 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 2003 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &faceangle1[i]
# 2003 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 2003 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                            ;
        b=
# 2004 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 2004 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &faceangle2[i]
# 2004 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 2004 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                            ;
        stepper3angle=3.6*a*(-1);
        stepper4angle=3*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, 
# 2009 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 2009 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, 
# 2010 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 2010 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueGenericSend( ( QueueHandle_Angle3 ), ( &stepper3angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
        xQueueGenericSend( ( QueueHandle_Angle4 ), ( &stepper4angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, ( TickType_t ) 0xffff);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, ( TickType_t ) 0xffff);
        }

        //4 clear
        stepper3finishflag=false;
        stepper4finishflag=false;
        vTaskDelete(TaskHandle_Stepper3);
        vTaskDelete(TaskHandle_Stepper4);
    }
    PenUp();

    //move to the left eye first point
    for(i=0; i<1; i++)
    {
        a=
# 2034 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 2034 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &lefteyeangle1[i]
# 2034 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 2034 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                               -
# 2034 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                                (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 2034 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                &faceangle1[i]
# 2034 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                                )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 2034 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                                                   ;
        b=
# 2035 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 2035 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &lefteyeangle2[i]
# 2035 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 2035 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                               -
# 2035 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                                (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 2035 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                &faceangle2[i]
# 2035 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                                )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 2035 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                                                   ;
        stepper3angle=3.6*a*(-1);
        stepper4angle=3*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, 
# 2040 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 2040 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, 
# 2041 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 2041 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueGenericSend( ( QueueHandle_Angle3 ), ( &stepper3angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
        xQueueGenericSend( ( QueueHandle_Angle4 ), ( &stepper4angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, ( TickType_t ) 0xffff);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, ( TickType_t ) 0xffff);
        }

        //4 clear
        stepper3finishflag=false;
        stepper4finishflag=false;
        vTaskDelete(TaskHandle_Stepper3);
        vTaskDelete(TaskHandle_Stepper4);
    }
    PenDown();
    //draw left eye
    for(i=1; i<42 /*3mm*/; i++)
    {
        a=
# 2064 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 2064 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &lefteyeangle1[i]
# 2064 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 2064 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                               ;
        b=
# 2065 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 2065 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &lefteyeangle2[i]
# 2065 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 2065 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                               ;
        stepper3angle=3.6*a*(-1);
        stepper4angle=3*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, 
# 2070 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 2070 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, 
# 2071 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 2071 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueGenericSend( ( QueueHandle_Angle3 ), ( &stepper3angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
        xQueueGenericSend( ( QueueHandle_Angle4 ), ( &stepper4angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, ( TickType_t ) 0xffff);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, ( TickType_t ) 0xffff);
        }

        //4 clear
        stepper3finishflag=false;
        stepper4finishflag=false;
        vTaskDelete(TaskHandle_Stepper3);
        vTaskDelete(TaskHandle_Stepper4);
    }
    PenUp();

    //move to the right eye first point
    for(i=0; i<1; i++)
    {
        a=
# 2095 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 2095 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &righteyeangle1[i]
# 2095 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 2095 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                -
# 2095 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                                 (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 2095 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                 &lefteyeangle1[i]
# 2095 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                                 )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 2095 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                                                       ;
        b=
# 2096 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 2096 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &righteyeangle2[i]
# 2096 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 2096 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                -
# 2096 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                                 (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 2096 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                 &lefteyeangle2[i]
# 2096 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                                 )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 2096 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                                                       ;
        stepper3angle=3.6*a*(-1);
        stepper4angle=3*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, 
# 2101 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 2101 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, 
# 2102 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 2102 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueGenericSend( ( QueueHandle_Angle3 ), ( &stepper3angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
        xQueueGenericSend( ( QueueHandle_Angle4 ), ( &stepper4angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, ( TickType_t ) 0xffff);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, ( TickType_t ) 0xffff);
        }

        //4 clear
        stepper3finishflag=false;
        stepper4finishflag=false;
        vTaskDelete(TaskHandle_Stepper3);
        vTaskDelete(TaskHandle_Stepper4);
    }
    PenDown();
    for(i=1; i<42 /*3mm*/; i++)
    {
        a=
# 2124 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 2124 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &righteyeangle1[i]
# 2124 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 2124 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                ;
        b=
# 2125 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 2125 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &righteyeangle2[i]
# 2125 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 2125 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                ;
        stepper3angle=3.6*a*(-1);
        stepper4angle=3*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, 
# 2130 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 2130 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, 
# 2131 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 2131 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueGenericSend( ( QueueHandle_Angle3 ), ( &stepper3angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
        xQueueGenericSend( ( QueueHandle_Angle4 ), ( &stepper4angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, ( TickType_t ) 0xffff);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, ( TickType_t ) 0xffff);
        }

        //4 clear
        stepper3finishflag=false;
        stepper4finishflag=false;
        vTaskDelete(TaskHandle_Stepper3);
        vTaskDelete(TaskHandle_Stepper4);
    }
    PenUp();

    //move to the mouth first point
    for(i=0; i<1; i++)
    {
        a=
# 2155 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 2155 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &mouthangle1[i]
# 2155 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 2155 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                             -
# 2155 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                              (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 2155 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                              &righteyeangle1[i]
# 2155 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                              )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 2155 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                                                     ;
        b=
# 2156 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 2156 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &mouthangle2[i]
# 2156 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 2156 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                             -
# 2156 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                              (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 2156 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                              &righteyeangle2[i]
# 2156 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                              )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 2156 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                                                     ;
        stepper3angle=3.6*a*(-1);
        stepper4angle=3*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, 
# 2161 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 2161 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, 
# 2162 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 2162 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueGenericSend( ( QueueHandle_Angle3 ), ( &stepper3angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
        xQueueGenericSend( ( QueueHandle_Angle4 ), ( &stepper4angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, ( TickType_t ) 0xffff);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, ( TickType_t ) 0xffff);
        }

        //4 clear
        stepper3finishflag=false;
        stepper4finishflag=false;
        vTaskDelete(TaskHandle_Stepper3);
        vTaskDelete(TaskHandle_Stepper4);
    }
    PenDown();
    for(i=1; i<60 /*3mm*/; i++)
    {
        a=
# 2184 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 2184 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &mouthangle1[i]
# 2184 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 2184 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                             ;
        b=
# 2185 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 2185 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &mouthangle2[i]
# 2185 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 2185 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                             ;
        stepper3angle=3.6*a*(-1);
        stepper4angle=3*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, 
# 2190 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 2190 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, 
# 2191 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 2191 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueGenericSend( ( QueueHandle_Angle3 ), ( &stepper3angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
        xQueueGenericSend( ( QueueHandle_Angle4 ), ( &stepper4angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, ( TickType_t ) 0xffff);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, ( TickType_t ) 0xffff);
        }

        //4 clear
        stepper3finishflag=false;
        stepper4finishflag=false;
        vTaskDelete(TaskHandle_Stepper3);
        vTaskDelete(TaskHandle_Stepper4);
    }
    PenUp();

    //move to the reset place
    for(i=0; i<1; i++)
    {
        a=
# 2215 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 2215 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &mouthangle1[i]
# 2215 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 2215 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                             *(-1);
        b=
# 2216 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 2216 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &mouthangle2[i]
# 2216 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 2216 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                             *(-1);
        stepper3angle=3.6*a*(-1);
        stepper4angle=3*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, 
# 2221 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 2221 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, 
# 2222 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 2222 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueGenericSend( ( QueueHandle_Angle3 ), ( &stepper3angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
        xQueueGenericSend( ( QueueHandle_Angle4 ), ( &stepper4angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, ( TickType_t ) 0xffff);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, ( TickType_t ) 0xffff);
        }

        //4 clear
        stepper3finishflag=false;
        stepper4finishflag=false;
        vTaskDelete(TaskHandle_Stepper3);
        vTaskDelete(TaskHandle_Stepper4);
    }
}
void RobotDrawLogo(void)
{
    float stepper3angle, stepper4angle;
    bool stepper3finishflag=false;
    bool stepper4finishflag=false;
    int i;
    float a,b;

    vTaskDelay(1000/( (TickType_t) 
# 2250 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                   (1 << (0 
# 2250 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                   /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 2250 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                   )) 
# 2250 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                   ));

    for(i=0; i<197 /*3mm*/; i++)
    {
        a=
# 2254 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 2254 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &logoangle1[i]
# 2254 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 2254 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                            ;
        b=
# 2255 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 2255 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
         &logoangle2[i]
# 2255 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
         )); float __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z+" "\n\t" "lpm %C0, Z+" "\n\t" "lpm %D0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 2255 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                            ;
        stepper3angle=3.6*a*(-1);
        stepper4angle=3*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, 
# 2260 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 2260 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, 
# 2261 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3 4
                                                        __null
# 2261 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                                            , 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueGenericSend( ( QueueHandle_Angle3 ), ( &stepper3angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );
        xQueueGenericSend( ( QueueHandle_Angle4 ), ( &stepper4angle ), ( ( TickType_t ) 0xffff ), ( ( BaseType_t ) 0 ) );

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, ( TickType_t ) 0xffff);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, ( TickType_t ) 0xffff);
        }

        //4 clear
        stepper3finishflag=false;
        stepper4finishflag=false;
        vTaskDelete(TaskHandle_Stepper3);
        vTaskDelete(TaskHandle_Stepper4);

        if(i==0)
        {
            servo2.attach(Pin_Servo2);
            vTaskDelay(500/( (TickType_t) 
# 2283 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         (1 << (0 
# 2283 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 2283 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         )) 
# 2283 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         ));
            ServoRotate(2, 0);
            vTaskDelay(500/( (TickType_t) 
# 2285 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         (1 << (0 
# 2285 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 2285 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         )) 
# 2285 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         ));
            Serial.println("Pen down!");
        }

        if(i==197 /*3mm*/-1-1)
        {
            servo2.attach(Pin_Servo2);
            vTaskDelay(500/( (TickType_t) 
# 2292 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         (1 << (0 
# 2292 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 2292 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         )) 
# 2292 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         ));
            ServoRotate(2, 1);
            vTaskDelay(500/( (TickType_t) 
# 2294 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         (1 << (0 
# 2294 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 2294 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                         )) 
# 2294 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                         ));
            Serial.println("Pen up!\n");
        }

        // vTaskDelay(20/portTICK_PERIOD_MS);
    }
}

// MP3
void Mp3Init(void)
{
    Mp3Serial.begin(9600);
    Mp3ChangeVolume(10);

    Serial.println("MP3 module has been initialized! ");
    delay(10);
}
void Mp3ChangeVolume(char level)
{
    int checkSum = 0;
    Mp3Serial.print((char)0xAA);
    Mp3Serial.print((char)0x13);
    Mp3Serial.print((char)0x01);
    Mp3Serial.print((char)level);
    checkSum = 0xAA+0x13+0x01+level;
    Mp3Serial.print((char)(checkSum & 0xFF));
}
// MP3 execute
void Mp3Execute(int index)
{
    int checkSum = 0;
    Mp3Serial.print((char)0xAA);
    Mp3Serial.print((char)0x08);
    Mp3Serial.print((char)0x0B);
    Mp3Serial.print((char)0x02);
    Mp3Serial.print((char)0x2F);
    Mp3Serial.print((char)(0x30+(index%100000)/10000));
    Mp3Serial.print((char)(0x30+(index%10000)/1000));
    Mp3Serial.print((char)(0x30+(index%1000)/100));
    Mp3Serial.print((char)(0x30+(index%100)/10));
    Mp3Serial.print((char)(0x30+(index%10)));
    Mp3Serial.print((char)0x2A);
    Mp3Serial.print((char)0x3F);
    Mp3Serial.print((char)0x3F);
    Mp3Serial.print((char)0x3F);
    checkSum = 0x2C5 + (index%100000)/10000 + (index%10000)/1000 + (index%1000)/100 + (index%100)/10 + index%10;
    Mp3Serial.print((char)(checkSum & 0xFF));

    vTaskDelay(4000/( (TickType_t) 
# 2342 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                   (1 << (0 
# 2342 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                   /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 2342 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                   )) 
# 2342 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                   ));
}
void RandomInit(void)
{
    randomSeed(analogRead(Pin_Random));
}
void MP3RandomGoExecute(void)
{
    int randomnumber=random(10)+1; //1-10

    Serial.print("randomnumber is "); Serial.println(randomnumber);
    switch(randomnumber)
    {
        case 1:
            Mp3Execute(33);
            break;
        case 2:
            Mp3Execute(34);
            break;
        case 3:
            Mp3Execute(35);
            break;
        case 4:
            Mp3Execute(34);
            break;
        case 5:
            Mp3Execute(35);
            break;
        default:
            break;
    }
}

// 1 magnet
void MagnetInit(void)
{
    pinMode(Pin_Magnet1, 0x1);

    digitalWrite(Pin_Magnet1, 0x0);

    Serial.println("Magnet module has been initialized! ");
    delay(10);
}
void MagnetSwitch(uint8_t num, bool magnetswitch)
{
    if(num==1 && magnetswitch==1)
    {
        digitalWrite(Pin_Magnet1, 0x1);
    }
    else if(num==1 && magnetswitch==0)
    {
        digitalWrite(Pin_Magnet1, 0x0);
    }
}

// 2 servo motors
void ServoInit(void)
{
    servo1.attach(Pin_Servo1);
    servo1.write(Servo1MaxAngle);
    servo2.attach(Pin_Servo2);

    servo2.write(Servo2MaxAngle);
    delay(200);
    servo1.detach();
    servo2.detach();
    delay(200);

    Serial.println("Servo motor module has been initialized! ");
    delay(10);
}
void ServoRotate(uint8_t num, bool servostatus)
{
    if(num==1 && servostatus==1)
    {
        for(int i=Servo1MinAngle; i<Servo1MaxAngle; i++)
        {
            servo1.write(i);
            vTaskDelay(16/( (TickType_t) 
# 2420 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                         (1 << (0 
# 2420 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 2420 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                         )) 
# 2420 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                         ));
        }
    }
    else if(num==1 && servostatus==0)
    {
        for(int i=Servo1MaxAngle; i>Servo1MinAngle; i--)
        {
            servo1.write(i);
            vTaskDelay(16/( (TickType_t) 
# 2428 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                         (1 << (0 
# 2428 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 2428 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                         )) 
# 2428 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                         ));
        }
    }

    if(num==2 && servostatus==1)
    {
        for(int i=Servo2MinAngle; i<Servo2MaxAngle; i++)
        {
            servo2.write(i);
            vTaskDelay(16/( (TickType_t) 
# 2437 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                         (1 << (0 
# 2437 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 2437 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                         )) 
# 2437 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                         ));
        }
    }
    else if(num==2 && servostatus==0)
    {
        for(int i=Servo2MaxAngle; i>Servo2MinAngle; i--)
        {
            servo2.write(i);
            vTaskDelay(16/( (TickType_t) 
# 2445 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                         (1 << (0 
# 2445 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                         /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 2445 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                         )) 
# 2445 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                         ));
        }
    }
}
void PenDown(void)
{
    servo2.attach(Pin_Servo2);
    vTaskDelay(500/( (TickType_t) 
# 2452 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 (1 << (0 
# 2452 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 2452 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 )) 
# 2452 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 ));
    ServoRotate(2, 0);
    vTaskDelay(500/( (TickType_t) 
# 2454 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 (1 << (0 
# 2454 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 2454 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 )) 
# 2454 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 ));
    Serial.println("Pen down!");
}
void PenUp(void)
{
    servo2.attach(Pin_Servo2);
    vTaskDelay(500/( (TickType_t) 
# 2460 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 (1 << (0 
# 2460 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 2460 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 )) 
# 2460 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 ));
    ServoRotate(2, 1);
    vTaskDelay(500/( (TickType_t) 
# 2462 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 (1 << (0 
# 2462 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 2462 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino" 3
                                 )) 
# 2462 "e:\\Z SenseTime 商汤科技 实习\\V0.4\\1 Arduino Code\\V0.4.2 Drawing\\G4VXE6\\G4VXE6.ino"
                                 ));
    Serial.println("Pen up!\n");
}
