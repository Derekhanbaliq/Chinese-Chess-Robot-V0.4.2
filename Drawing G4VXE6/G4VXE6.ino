/*
 * Author       :  Derek Zhou
 * Date         :  06/08/2021
 * Version      :  Generation 4 Version X Pattern 6
 * New Function :  Logo Test with new pin config
 */

#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <FlexiTimer2.h>
#include <avr/pgmspace.h>
#include "anglearray.h"

void TaskMain(void *pvParameters);
void TaskStepper1(void *pvParameters);
void TaskStepper2(void *pvParameters);
void TaskStepper3(void *pvParameters);
void TaskStepper4(void *pvParameters);

#define OFF  0
#define ON   1
#define UP   1
#define DOWN 0
#define TRANSMISSION_RATIO_1   3.6
#define TRANSMISSION_RATIO_2   3
#define PULSE_PER_REVOLUTION   1600 //θ=1.8°, microstep=8, ∴360°÷θ×microstep=1600
#define CHESS_WAITING_TIME     500

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
QueueHandle_t  QueueHandle_Angle1, QueueHandle_Angle2, QueueHandle_Finish1, QueueHandle_Finish2;
TaskControlBlock_t *TaskHandle_Stepper3, *TaskHandle_Stepper4;
QueueHandle_t  QueueHandle_Angle3, QueueHandle_Angle4, QueueHandle_Finish3, QueueHandle_Finish4;
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
extern const float circle5mmangle1[CIRCLE_5MM_LEN] PROGMEM;
extern const float circle5mmangle2[CIRCLE_5MM_LEN] PROGMEM;
extern const float circle3mmangle1[CIRCLE_3MM_LEN] PROGMEM;
extern const float circle3mmangle2[CIRCLE_3MM_LEN] PROGMEM;
extern const float circle2mmangle1[CIRCLE_2MM_LEN] PROGMEM;
extern const float circle2mmangle2[CIRCLE_2MM_LEN] PROGMEM;
extern const float circle1mmangle1[CIRCLE_1MM_LEN] PROGMEM;
extern const float circle1mmangle2[CIRCLE_1MM_LEN] PROGMEM;
extern const float lineangle1[LINE_LEN] PROGMEM;
extern const float lineangle2[LINE_LEN] PROGMEM;
extern const float squareangle1[SQUARE_LEN] PROGMEM;
extern const float squareangle2[SQUARE_LEN] PROGMEM;
extern const float faceangle1[FACE_LEN] PROGMEM;
extern const float faceangle2[FACE_LEN] PROGMEM;
extern const float lefteyeangle1[LEFTEYE_LEN] PROGMEM;
extern const float lefteyeangle2[LEFTEYE_LEN] PROGMEM;
extern const float righteyeangle1[RIGHTEYE_LEN] PROGMEM;
extern const float righteyeangle2[RIGHTEYE_LEN] PROGMEM;
extern const float mouthangle1[MOUTH_LEN] PROGMEM;
extern const float mouthangle2[MOUTH_LEN] PROGMEM;
extern const float logoangle1[LOGO_LEN] PROGMEM;
extern const float logoangle2[LOGO_LEN] PROGMEM;
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

    QueueHandle_Angle1=xQueueCreate(1, sizeof(float));
    QueueHandle_Angle2=xQueueCreate(1, sizeof(float));
    QueueHandle_Angle3=xQueueCreate(1, sizeof(float));
    QueueHandle_Angle4=xQueueCreate(1, sizeof(float));
    QueueHandle_Finish1=xQueueCreate(1, sizeof(bool));
    QueueHandle_Finish2=xQueueCreate(1, sizeof(bool));
    QueueHandle_Finish3=xQueueCreate(1, sizeof(bool));
    QueueHandle_Finish4=xQueueCreate(1, sizeof(bool));
    Serial.println("Info queues have been created! ");

    xTaskCreate(TaskMain, "Main", 1024, NULL, 2, NULL); //can also be recognized as protocol+led+button task
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

    // robot is ready
    vTaskDelay(500/portTICK_PERIOD_MS);
    // Mp3Execute(32);
    RobotIsReady();
    // Mp3Execute(31);
    vTaskDelay(500/portTICK_PERIOD_MS);

    /*
    //start
    while(GameStartFlag==false) //robot wait the player to press the start button
    {
        ButtonStartScan(); //contains CheckStartOk();
    }
    */

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

        RobotDrawLogo();
        
        vTaskDelay(5000/portTICK_PERIOD_MS);
    }
}
void TaskStepper1(void *pvParameters)
{
    (void) pvParameters;

    float Stepper1Angle;
    bool Stepper1Finish=false;

    while(1)
    {
        if(xQueueReceive(QueueHandle_Angle1, &Stepper1Angle, portMAX_DELAY))
        {
            // digitalWrite(Pin_Stepper1Enable, LOW);
            TaskStepper1_AngleToPulseAndRotate(Stepper1Angle);
            // digitalWrite(Pin_Stepper1Enable, HIGH);
        }
        Stepper1Finish=true;
        xQueueSend(QueueHandle_Finish1, &Stepper1Finish, portMAX_DELAY);
        Stepper1Angle=0;
        Stepper1Finish=false;
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}
void TaskStepper2(void *pvParameters)
{
    (void) pvParameters;

    float Stepper2Angle;
    bool Stepper2Finish=false;

    while(1)
    {
        if(xQueueReceive(QueueHandle_Angle2, &Stepper2Angle, portMAX_DELAY))
        {
            // digitalWrite(Pin_Stepper2Enable, LOW);
            TaskStepper2_AngleToPulseAndRotate(Stepper2Angle);
            // digitalWrite(Pin_Stepper2Enable, HIGH);
        }
        Stepper2Finish=true;
        xQueueSend(QueueHandle_Finish2, &Stepper2Finish, portMAX_DELAY);
        Stepper2Angle=0;
        Stepper2Finish=false;
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}
void TaskStepper3(void *pvParameters)
{
    (void) pvParameters;

    float Stepper3Angle;
    bool Stepper3Finish=false;

    while(1)
    {
        if(xQueueReceive(QueueHandle_Angle3, &Stepper3Angle, portMAX_DELAY))
        {
            TaskStepper3_AngleToPulseAndRotate(Stepper3Angle);
        }
        Stepper3Finish=true;
        xQueueSend(QueueHandle_Finish3, &Stepper3Finish, portMAX_DELAY);
        Stepper3Angle=0;
        Stepper3Finish=false;
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}
void TaskStepper4(void *pvParameters)
{
    (void) pvParameters;

    float Stepper4Angle;
    bool Stepper4Finish=false;

    while(1)
    {
        if(xQueueReceive(QueueHandle_Angle4, &Stepper4Angle, portMAX_DELAY))
        {
            TaskStepper4_AngleToPulseAndRotate(Stepper4Angle);
        }
        Stepper4Finish=true;
        xQueueSend(QueueHandle_Finish4, &Stepper4Finish, portMAX_DELAY);
        Stepper4Angle=0;
        Stepper4Finish=false;
        vTaskDelay(10/portTICK_PERIOD_MS);
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
			vTaskDelay(1/portTICK_PERIOD_MS);
			maxWaitTimeMs--;
		}
		//确定有数据后，再等2ms，确保#ok全部收完
		if(maxWaitTimeMs==0)
        {
		    vTaskDelay(2/portTICK_PERIOD_MS);
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
        vTaskDelay(10/portTICK_PERIOD_MS); //wait the input char to be totally received into the buffer
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

    vTaskDelay(10/portTICK_PERIOD_MS);
}
void CheckFaceRecognition(void)
{
    while(Serial3.available() && stringcomplete==false) //jump out as long as "true"
    {
        globalinchar=(char)Serial3.read();
        Serial3ByteTranslate(globalinchar);
        vTaskDelay(10/portTICK_PERIOD_MS); //wait the input char to be totally received into the buffer
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

    vTaskDelay(10/portTICK_PERIOD_MS);
}
void CheckStartOk(void)
{
    while(Serial3.available() && stringcomplete==false) //jump out as long as "true"
    {
        globalinchar=(char)Serial3.read();
        Serial3ByteTranslate(globalinchar);
        vTaskDelay(10/portTICK_PERIOD_MS); //wait the input char to be totally received into the buffer
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
                LedSwitch(Pin_LedGreen, ON);
                StartOkFlag=true;
            }
        }
        else
        {
            Serial.println("Sorry, only receive #startok right now~ ");
        }
        ClearOut();
    }

    vTaskDelay(10/portTICK_PERIOD_MS);
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
    else if((keyword=="move" || keyword=="eat" || keyword=="movec" || keyword=="eatc" || keyword=="movew" ||keyword=="eatw") \
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
    if(keyword=="initok" || keyword=="move" || keyword=="eat" || keyword=="movec" || keyword=="eatc" || keyword=="movew" || \
                        keyword=="eatw" || keyword=="result" || keyword=="warn" || keyword=="error" || keyword=="retraok" || \
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
            LedSwitch(Pin_LedGreen, ON);
        }
        else if(keyword=="eat")
        {
            Serial.println("EAT! ");
            CoordinateDataToArray(keyword, coordinatestring);
            Serial0PrintMoveOrEat(keyword);
            Mp3Execute(3);
            RobotEat();
            Mp3Execute(4);
            LedSwitch(Pin_LedGreen, ON);
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
            LedSwitch(Pin_LedGreen, ON);
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
            LedSwitch(Pin_LedGreen, ON);
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
            LedSwitch(Pin_LedBlue, ON);
            vTaskDelay(200/portTICK_PERIOD_MS);
            LedSwitch(Pin_LedBlue, OFF);  
            vTaskDelay(200/portTICK_PERIOD_MS);
        }
        LedSwitch(Pin_LedGreen, OFF);
        LedSwitch(Pin_LedBlue, ON);
    }
    else if(result=="win")
    {
        Serial.println("Sorry, you lose... ");
        Mp3Execute(7);
        LedSwitch(Pin_LedGreen, OFF);
        LedSwitch(Pin_LedBlue, ON);
    }
    else if(result=="draw")
    {
        Serial.println("Oh, draw~ ");
        Mp3Execute(8);
        for(int i=0; i<5; i++)
        {
            LedSwitch(Pin_LedBlue, ON);
            vTaskDelay(200/portTICK_PERIOD_MS);
            LedSwitch(Pin_LedBlue, OFF);  
            vTaskDelay(200/portTICK_PERIOD_MS);
        }
        LedSwitch(Pin_LedGreen, OFF);
        LedSwitch(Pin_LedBlue, ON);
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
        LedSwitch(Pin_LedRed, ON);
        vTaskDelay(500/portTICK_PERIOD_MS);
        LedSwitch(Pin_LedRed, OFF); // equals refresh in each loop
    }
    else
    {
        LedSwitch(Pin_LedRed, OFF);
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
    pinMode(Pin_LedGreen, OUTPUT);
    pinMode(Pin_LedBlue, OUTPUT);
    pinMode(Pin_LedRed, OUTPUT);
  
    for(int i=0; i<5; i++)
    {
        LedSwitch(Pin_LedGreen, ON);
        LedSwitch(Pin_LedBlue, ON);
        LedSwitch(Pin_LedRed, ON);
        delay(100);
        LedSwitch(Pin_LedGreen, OFF);
        LedSwitch(Pin_LedBlue, OFF);
        LedSwitch(Pin_LedRed, OFF);
        delay(100);
    }
    LedSwitch(Pin_LedGreen, OFF);
    LedSwitch(Pin_LedBlue, ON);
    LedSwitch(Pin_LedRed, OFF);

    Serial.println("LED module has been initialized! ");
    delay(10);
}
void LedSwitch(uint8_t pin, bool status)
{
    if(status==ON)
    {
        digitalWrite(pin, HIGH); 
    }
    else if(status==OFF)
    {
        digitalWrite(pin, LOW); 
    }
}

// button
void ButtonInit(void)
{
    pinMode(Pin_ButtonGo, INPUT);
    pinMode(Pin_ButtonStart, INPUT);
    pinMode(Pin_ButtonRegret, INPUT);

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
                LedSwitch(Pin_LedBlue, OFF);
                vTaskDelay(200/portTICK_PERIOD_MS);
                digitalWrite(Pin_LedBlue, ON);  
                vTaskDelay(200/portTICK_PERIOD_MS);
            }
            LedSwitch(Pin_LedBlue, OFF);
            RobotHandshake("#start\n");
            while(StartOkFlag==false)
            {
                CheckStartOk();
            }
            Mp3Execute(2);
            LedSwitch(Pin_LedGreen, ON);
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
            LedSwitch(Pin_LedGreen, OFF);
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
                LedSwitch(Pin_LedGreen, ON);
                vTaskDelay(200/portTICK_PERIOD_MS);
                LedSwitch(Pin_LedGreen, OFF); 
                vTaskDelay(200/portTICK_PERIOD_MS);
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
    static bool pulseflag1 = HIGH;
    static bool pulseflag2 = HIGH;
    static bool pulseflag3 = HIGH;
    static bool pulseflag4 = HIGH;

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
    pinMode(Pin_Stepper1Direction, OUTPUT);
    pinMode(Pin_Stepper1Pulse, OUTPUT);
    pinMode(Pin_Stepper1Enable, OUTPUT);
    pinMode(Pin_Stepper2Direction, OUTPUT);
    pinMode(Pin_Stepper2Pulse, OUTPUT);
    pinMode(Pin_Stepper2Enable, OUTPUT);
    pinMode(Pin_Stepper3Direction, OUTPUT);
    pinMode(Pin_Stepper3Pulse, OUTPUT);
    pinMode(Pin_Stepper3Enable, OUTPUT);
    pinMode(Pin_Stepper4Direction, OUTPUT);
    pinMode(Pin_Stepper4Pulse, OUTPUT);
    pinMode(Pin_Stepper4Enable, OUTPUT);

    digitalWrite(Pin_Stepper1Enable, LOW);
    digitalWrite(Pin_Stepper2Enable, LOW);
    digitalWrite(Pin_Stepper3Enable, LOW);
    digitalWrite(Pin_Stepper4Enable, LOW); //work
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
    vTaskDelay(100/portTICK_PERIOD_MS);

    //to current coordinate
    //1 create task
    stepper1angle1=TRANSMISSION_RATIO_1*CoordinateArrayToAngle(x1, y1, 1);
    stepper2angle1=TRANSMISSION_RATIO_2*CoordinateArrayToAngle(x1, y1, 2);
    xTaskCreate(TaskStepper1, "Stepper1Rotate", 128, NULL, 1, &TaskHandle_Stepper1);
    xTaskCreate(TaskStepper2, "Stepper2Rotate", 128, NULL, 1, &TaskHandle_Stepper2);
    vTaskDelay(10/portTICK_PERIOD_MS);
    //2 transmit data
    xQueueSend(QueueHandle_Angle1, &stepper1angle1, portMAX_DELAY);
    xQueueSend(QueueHandle_Angle2, &stepper2angle1, portMAX_DELAY);
    vTaskDelay(10/portTICK_PERIOD_MS);
    //3 receive data
    while(stepper1finishflag==false || stepper2finishflag==false)
    {
        xQueueReceive(QueueHandle_Finish1, &stepper1finishflag, portMAX_DELAY);
        xQueueReceive(QueueHandle_Finish2, &stepper2finishflag, portMAX_DELAY);
    }
    vTaskDelay(10/portTICK_PERIOD_MS);
    //4 clear
    stepper1finishflag=false;
    stepper2finishflag=false;
    vTaskDelete(TaskHandle_Stepper1);
    vTaskDelete(TaskHandle_Stepper2);
    vTaskDelay(10/portTICK_PERIOD_MS);
    Serial.println("...");

    //catch the chess
    vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
    MagnetSwitch(1, ON);
    ServoRotate(1, DOWN);
    vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
    ServoRotate(1, UP);
    vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);

    //to target coordinate
    stepper1angle2=TRANSMISSION_RATIO_1*CoordinateArrayToAngle(x2, y2, 1);
    stepper2angle2=TRANSMISSION_RATIO_2*CoordinateArrayToAngle(x2, y2, 2);
    delta1=stepper1angle2-stepper1angle1;
    delta2=stepper2angle2-stepper2angle1;
    vTaskDelay(10/portTICK_PERIOD_MS);
    xTaskCreate(TaskStepper1, "Stepper1Rotate", 128, NULL, 1, &TaskHandle_Stepper1);
    xTaskCreate(TaskStepper2, "Stepper2Rotate", 128, NULL, 1, &TaskHandle_Stepper2);
    xQueueSend(QueueHandle_Angle1, &delta1, portMAX_DELAY);
    xQueueSend(QueueHandle_Angle2, &delta2, portMAX_DELAY);
    vTaskDelay(10/portTICK_PERIOD_MS);
    while(stepper1finishflag==false || stepper2finishflag==false)
    {
        xQueueReceive(QueueHandle_Finish1, &stepper1finishflag, portMAX_DELAY);
        xQueueReceive(QueueHandle_Finish2, &stepper2finishflag, portMAX_DELAY);
    }
    vTaskDelay(10/portTICK_PERIOD_MS);
    stepper1finishflag=false;
    stepper2finishflag=false;
    vTaskDelete(TaskHandle_Stepper1);
    vTaskDelete(TaskHandle_Stepper2);
    vTaskDelay(10/portTICK_PERIOD_MS);
    Serial.println("...");

    //set the chess
    vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
    ServoRotate(1, DOWN);
    MagnetSwitch(1, OFF);
    vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
    ServoRotate(1, UP);
    vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);

    //back to initial position
    return1=(-1)*stepper1angle2;
    return2=(-1)*stepper2angle2;
    xTaskCreate(TaskStepper1, "Stepper1Rotate", 128, NULL, 1, &TaskHandle_Stepper1);
    xTaskCreate(TaskStepper2, "Stepper2Rotate", 128, NULL, 1, &TaskHandle_Stepper2);
    vTaskDelay(10/portTICK_PERIOD_MS);
    xQueueSend(QueueHandle_Angle1, &return1, portMAX_DELAY);
    xQueueSend(QueueHandle_Angle2, &return2, portMAX_DELAY);
    vTaskDelay(10/portTICK_PERIOD_MS);
    while(stepper1finishflag==false || stepper2finishflag==false)
    {
        xQueueReceive(QueueHandle_Finish1, &stepper1finishflag, portMAX_DELAY);
        xQueueReceive(QueueHandle_Finish2, &stepper2finishflag, portMAX_DELAY);
    }
    vTaskDelay(10/portTICK_PERIOD_MS);
    stepper1finishflag=false;
    stepper2finishflag=false;
    vTaskDelete(TaskHandle_Stepper1);
    vTaskDelete(TaskHandle_Stepper2);
    vTaskDelay(10/portTICK_PERIOD_MS);
    Serial.println("Move step is done. ");

    servo1.detach();
    vTaskDelay(200/portTICK_PERIOD_MS);
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
    vTaskDelay(100/portTICK_PERIOD_MS);

    //to target coordinate
    stepper1angle2=TRANSMISSION_RATIO_1*CoordinateArrayToAngle(x2,y2,1);
    stepper2angle2=TRANSMISSION_RATIO_2*CoordinateArrayToAngle(x2,y2,2);
    xTaskCreate(TaskStepper1, "Stepper1Rotate", 128, NULL, 1, &TaskHandle_Stepper1);
    xTaskCreate(TaskStepper2, "Stepper2Rotate", 128, NULL, 1, &TaskHandle_Stepper2);
    vTaskDelay(10/portTICK_PERIOD_MS);
    xQueueSend(QueueHandle_Angle1, &stepper1angle2, portMAX_DELAY);
    xQueueSend(QueueHandle_Angle2, &stepper2angle2, portMAX_DELAY);
    vTaskDelay(10/portTICK_PERIOD_MS);
    while(stepper1finishflag==false || stepper2finishflag==false)
    {
        xQueueReceive(QueueHandle_Finish1, &stepper1finishflag, portMAX_DELAY);
        xQueueReceive(QueueHandle_Finish2, &stepper2finishflag, portMAX_DELAY);
    }
    vTaskDelay(10/portTICK_PERIOD_MS);
    stepper1finishflag=false;
    stepper2finishflag=false;
    vTaskDelete(TaskHandle_Stepper1);
    vTaskDelete(TaskHandle_Stepper2);
    vTaskDelay(10/portTICK_PERIOD_MS);
    Serial.println("...");

    //catch the chess
    vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
    MagnetSwitch(1, ON);
    ServoRotate(1, DOWN);
    vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
    ServoRotate(1, UP);
    vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);

    //move the chess out (back to initial position)
    return1=(-1)*stepper1angle2;
    return2=(-1)*stepper2angle2;
    xTaskCreate(TaskStepper1, "Stepper1Rotate", 128, NULL, 1, &TaskHandle_Stepper1);
    xTaskCreate(TaskStepper2, "Stepper2Rotate", 128, NULL, 1, &TaskHandle_Stepper2);
    vTaskDelay(10/portTICK_PERIOD_MS);
    xQueueSend(QueueHandle_Angle1, &return1, portMAX_DELAY);
    xQueueSend(QueueHandle_Angle2, &return2, portMAX_DELAY);
    vTaskDelay(10/portTICK_PERIOD_MS);
    while(stepper1finishflag==false || stepper2finishflag==false)
    {
        xQueueReceive(QueueHandle_Finish1, &stepper1finishflag, portMAX_DELAY);
        xQueueReceive(QueueHandle_Finish2, &stepper2finishflag, portMAX_DELAY);
    }
    vTaskDelay(10/portTICK_PERIOD_MS);
    stepper1finishflag=false;
    stepper2finishflag=false;
    vTaskDelete(TaskHandle_Stepper1);
    vTaskDelete(TaskHandle_Stepper2);
    vTaskDelay(10/portTICK_PERIOD_MS);
    Serial.println("...");

    //drop the chess
    vTaskDelay(CHESS_WAITING_TIME*2/portTICK_PERIOD_MS);
    MagnetSwitch(1, OFF);
    vTaskDelay(CHESS_WAITING_TIME*2/portTICK_PERIOD_MS);

    //to current coordinate
    stepper1angle1=TRANSMISSION_RATIO_1*CoordinateArrayToAngle(x1, y1, 1);
    stepper2angle1=TRANSMISSION_RATIO_2*CoordinateArrayToAngle(x1, y1, 2);
    xTaskCreate(TaskStepper1, "Stepper1Rotate", 128, NULL, 1, &TaskHandle_Stepper1);
    xTaskCreate(TaskStepper2, "Stepper2Rotate", 128, NULL, 1, &TaskHandle_Stepper2);
    vTaskDelay(10/portTICK_PERIOD_MS);
    xQueueSend(QueueHandle_Angle1, &stepper1angle1, portMAX_DELAY);
    xQueueSend(QueueHandle_Angle2, &stepper2angle1, portMAX_DELAY);
    vTaskDelay(10/portTICK_PERIOD_MS);
    while(stepper1finishflag==false || stepper2finishflag==false)
    {
        xQueueReceive(QueueHandle_Finish1, &stepper1finishflag, portMAX_DELAY);
        xQueueReceive(QueueHandle_Finish2, &stepper2finishflag, portMAX_DELAY);
    }
    vTaskDelay(10/portTICK_PERIOD_MS);
    stepper1finishflag=false;
    stepper2finishflag=false;
    vTaskDelete(TaskHandle_Stepper1);
    vTaskDelete(TaskHandle_Stepper2);
    vTaskDelay(10/portTICK_PERIOD_MS);
    Serial.println("...");

    //catch the chess
    vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
    MagnetSwitch(1, ON);
    ServoRotate(1, DOWN);
    vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
    ServoRotate(1, UP);
    vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);

    //to target coordinate
    stepper1angle2=TRANSMISSION_RATIO_1*CoordinateArrayToAngle(x2, y2, 1);
    stepper2angle2=TRANSMISSION_RATIO_2*CoordinateArrayToAngle(x2, y2, 2);
    delta1=stepper1angle2-stepper1angle1;
    delta2=stepper2angle2-stepper2angle1;
    xTaskCreate(TaskStepper1, "Stepper1Rotate", 128, NULL, 1, &TaskHandle_Stepper1);
    xTaskCreate(TaskStepper2, "Stepper2Rotate", 128, NULL, 1, &TaskHandle_Stepper2);
    vTaskDelay(10/portTICK_PERIOD_MS);
    xQueueSend(QueueHandle_Angle1, &delta1, portMAX_DELAY);
    xQueueSend(QueueHandle_Angle2, &delta2, portMAX_DELAY);
    vTaskDelay(10/portTICK_PERIOD_MS);
    while(stepper1finishflag==false || stepper2finishflag==false)
    {
        xQueueReceive(QueueHandle_Finish1, &stepper1finishflag, portMAX_DELAY);
        xQueueReceive(QueueHandle_Finish2, &stepper2finishflag, portMAX_DELAY);
    }
    vTaskDelay(10/portTICK_PERIOD_MS);
    stepper1finishflag=false;
    stepper2finishflag=false;
    vTaskDelete(TaskHandle_Stepper1);
    vTaskDelete(TaskHandle_Stepper2);
    vTaskDelay(10/portTICK_PERIOD_MS);
    Serial.println("...");

    //set the chess
    vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
    ServoRotate(1, DOWN);
    MagnetSwitch(1, OFF);
    vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
    ServoRotate(1, UP);
    vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);

    //back to initial position
    return1=(-1)*stepper1angle2;
    return2=(-1)*stepper2angle2;
    xTaskCreate(TaskStepper1, "Stepper1Rotate", 128, NULL, 1, &TaskHandle_Stepper1);
    xTaskCreate(TaskStepper2, "Stepper2Rotate", 128, NULL, 1, &TaskHandle_Stepper2);
    vTaskDelay(10/portTICK_PERIOD_MS);
    xQueueSend(QueueHandle_Angle1, &return1, portMAX_DELAY);
    xQueueSend(QueueHandle_Angle2, &return2, portMAX_DELAY);
    vTaskDelay(10/portTICK_PERIOD_MS);
    while(stepper1finishflag==false || stepper2finishflag==false)
    {
        xQueueReceive(QueueHandle_Finish1, &stepper1finishflag, portMAX_DELAY);
        xQueueReceive(QueueHandle_Finish2, &stepper2finishflag, portMAX_DELAY);
    }
    vTaskDelay(10/portTICK_PERIOD_MS);
    stepper1finishflag=false;
    stepper2finishflag=false;
    vTaskDelete(TaskHandle_Stepper1);
    vTaskDelete(TaskHandle_Stepper2);
    vTaskDelay(10/portTICK_PERIOD_MS);
    Serial.println("Eat step is done. ");

    servo1.detach();
    vTaskDelay(200/portTICK_PERIOD_MS);
}
void RobotIsReady(void)
{
    float stepper1angle1, stepper2angle1, stepper3angle1, stepper4angle1;
    bool stepper1finishflag=false;
    bool stepper2finishflag=false;
    bool stepper3finishflag=false;
    bool stepper4finishflag=false;

    //1 create task
    stepper1angle1=TRANSMISSION_RATIO_1*90;
    stepper2angle1=TRANSMISSION_RATIO_2*90;
    stepper3angle1=TRANSMISSION_RATIO_1*90;
    stepper4angle1=TRANSMISSION_RATIO_2*(-90);
    xTaskCreate(TaskStepper1, "Stepper1Rotate", 128, NULL, 1, &TaskHandle_Stepper1);
    xTaskCreate(TaskStepper2, "Stepper2Rotate", 128, NULL, 1, &TaskHandle_Stepper2);
    xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, NULL, 1, &TaskHandle_Stepper3);
    xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, NULL, 1, &TaskHandle_Stepper4);
    vTaskDelay(10/portTICK_PERIOD_MS);
    //2 transmit data
    xQueueSend(QueueHandle_Angle1, &stepper1angle1, portMAX_DELAY);
    xQueueSend(QueueHandle_Angle2, &stepper2angle1, portMAX_DELAY);
    xQueueSend(QueueHandle_Angle3, &stepper3angle1, portMAX_DELAY);
    xQueueSend(QueueHandle_Angle4, &stepper4angle1, portMAX_DELAY);
    vTaskDelay(10/portTICK_PERIOD_MS);
    //3 receive data
    while(stepper1finishflag==false || stepper2finishflag==false || stepper3finishflag==false || stepper4finishflag==false)
    {
        xQueueReceive(QueueHandle_Finish1, &stepper1finishflag, portMAX_DELAY);
        xQueueReceive(QueueHandle_Finish2, &stepper2finishflag, portMAX_DELAY);
        xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, portMAX_DELAY);
        xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, portMAX_DELAY);
    }
    vTaskDelay(10/portTICK_PERIOD_MS);
    //4 clear
    stepper1finishflag=false;
    stepper2finishflag=false;
    stepper3finishflag=false;
    stepper4finishflag=false;
    vTaskDelete(TaskHandle_Stepper1);
    vTaskDelete(TaskHandle_Stepper2);
    vTaskDelete(TaskHandle_Stepper3);
    vTaskDelete(TaskHandle_Stepper4);
    vTaskDelay(10/portTICK_PERIOD_MS);
}
// transform the angle to pulse  --Dividing the function for each stepper in order to transplant the FreeRTOS
void TaskStepper1_AngleToPulseAndRotate(float angle)
{   
    //judge the direction
    if(angle>=0)
    {
        digitalWrite(Pin_Stepper1Direction, HIGH); //anti-clockwise! 
    }
    else
    {
        digitalWrite(Pin_Stepper1Direction, LOW);
    }

    //calculate the pulse number
    pulsenumber1=2*abs(angle)*PULSE_PER_REVOLUTION/360.00; //toggle!
    Serial.print("Stepper 1 pulse: ");
    Serial.println(pulsenumber1);
    // Serial.println("..."); //only stepper2 need ...!

    //execute the rotation
    while(pulsecnt1<pulsenumber1) //toggle!
    { 
        vTaskDelay(20/portTICK_PERIOD_MS);
    }
    pulsecnt1=0;
    pulsenumber1=0;
}
void TaskStepper2_AngleToPulseAndRotate(float angle)
{
    //judge the direction
    if(angle>=0)
    {
        digitalWrite(Pin_Stepper2Direction, LOW); //anti-clockwise! opposite!!!!
    }
    else
    {
        digitalWrite(Pin_Stepper2Direction, HIGH);
    }

    //calculate the pulse number
    pulsenumber2=2*abs(angle)*PULSE_PER_REVOLUTION/360.00; //toggle!
    Serial.print("Stepper 2 pulse: ");
    Serial.println(pulsenumber2);

    //execute the rotation
    while(pulsecnt2<pulsenumber2) 
    {   
        vTaskDelay(20/portTICK_PERIOD_MS);
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
        digitalWrite(Pin_Stepper3Direction, LOW); //anti-clockwise! opposite!!!!
        angle+=missangle3;
    }
    else
    {
        digitalWrite(Pin_Stepper3Direction, HIGH);
        angle-=missangle3;
    }

    //calculate the pulse number
    pulsenumber3=2*abs(angle)*PULSE_PER_REVOLUTION/360.00; //toggle!
    realangle3=pulsenumber3/2*360.00/PULSE_PER_REVOLUTION;
    missangle3=abs(angle)-abs(realangle3);

    //execute the rotation
    while(pulsecnt3<pulsenumber3) 
    {   
        vTaskDelay(20/portTICK_PERIOD_MS);
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
        digitalWrite(Pin_Stepper4Direction, LOW); //anti-clockwise! opposite!!!!
        angle+=missangle4;
    }
    else
    {
        digitalWrite(Pin_Stepper4Direction, HIGH);
        angle-=missangle4;
    }

    //calculate the pulse number
    pulsenumber4=2*abs(angle)*PULSE_PER_REVOLUTION/360.00; //toggle!
    realangle4=pulsenumber4/2*360.00/PULSE_PER_REVOLUTION;
    missangle4=abs(angle)-abs(realangle4);
    // Serial.print("pulsenumber4="); Serial.println(pulsenumber4);
    // Serial.print("realangle4="); Serial.println(realangle4);
    // Serial.print("missangle4="); Serial.println(missangle4);

    //execute the rotation
    while(pulsecnt4<pulsenumber4) 
    {   
        vTaskDelay(20/portTICK_PERIOD_MS);
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

    vTaskDelay(1000/portTICK_PERIOD_MS);

    for(i=0; i<CIRCLE_5MM_LEN; i++)
    {
        a=pgm_read_float_near(&circle5mmangle1[i]);
        b=pgm_read_float_near(&circle5mmangle2[i]);
        stepper3angle=TRANSMISSION_RATIO_1*a*(-1);
        stepper4angle=TRANSMISSION_RATIO_2*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, NULL, 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, NULL, 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueSend(QueueHandle_Angle3, &stepper3angle, portMAX_DELAY);
        xQueueSend(QueueHandle_Angle4, &stepper4angle, portMAX_DELAY);

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, portMAX_DELAY);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, portMAX_DELAY);
        }

        //4 clear
        stepper3finishflag=false;
        stepper4finishflag=false;
        vTaskDelete(TaskHandle_Stepper3);
        vTaskDelete(TaskHandle_Stepper4);

        if(i==0)
        {
            servo2.attach(Pin_Servo2);
            vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
            ServoRotate(2, DOWN);
            vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
            Serial.println("Pen down!");
        }

        if(i==CIRCLE_5MM_LEN-1-1)
        {
            servo2.attach(Pin_Servo2);
            vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
            ServoRotate(2, UP);
            vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
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

    vTaskDelay(1000/portTICK_PERIOD_MS);

    for(i=0; i<CIRCLE_3MM_LEN; i++)
    {
        a=pgm_read_float_near(&circle3mmangle1[i]);
        b=pgm_read_float_near(&circle3mmangle2[i]);
        stepper3angle=TRANSMISSION_RATIO_1*a*(-1);
        stepper4angle=TRANSMISSION_RATIO_2*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, NULL, 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, NULL, 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueSend(QueueHandle_Angle3, &stepper3angle, portMAX_DELAY);
        xQueueSend(QueueHandle_Angle4, &stepper4angle, portMAX_DELAY);

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, portMAX_DELAY);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, portMAX_DELAY);
        }

        //4 clear
        stepper3finishflag=false;
        stepper4finishflag=false;
        vTaskDelete(TaskHandle_Stepper3);
        vTaskDelete(TaskHandle_Stepper4);

        if(i==0)
        {
            servo2.attach(Pin_Servo2);
            vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
            ServoRotate(2, DOWN);
            vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
            Serial.println("Pen down!");
        }

        if(i==CIRCLE_3MM_LEN-1-1)
        {
            servo2.attach(Pin_Servo2);
            vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
            ServoRotate(2, UP);
            vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
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

    vTaskDelay(1000/portTICK_PERIOD_MS);

    for(i=0; i<CIRCLE_2MM_LEN; i++)
    {
        a=pgm_read_float_near(&circle2mmangle1[i]);
        b=pgm_read_float_near(&circle2mmangle2[i]);
        stepper3angle=TRANSMISSION_RATIO_1*a*(-1);
        stepper4angle=TRANSMISSION_RATIO_2*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, NULL, 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, NULL, 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueSend(QueueHandle_Angle3, &stepper3angle, portMAX_DELAY);
        xQueueSend(QueueHandle_Angle4, &stepper4angle, portMAX_DELAY);

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, portMAX_DELAY);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, portMAX_DELAY);
        }

        //4 clear
        stepper3finishflag=false;
        stepper4finishflag=false;
        vTaskDelete(TaskHandle_Stepper3);
        vTaskDelete(TaskHandle_Stepper4);

        if(i==0)
        {
            servo2.attach(Pin_Servo2);
            vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
            ServoRotate(2, DOWN);
            vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
            Serial.println("Pen down!");
        }

        if(i==CIRCLE_2MM_LEN-1-1)
        {
            servo2.attach(Pin_Servo2);
            vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
            ServoRotate(2, UP);
            vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
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

    vTaskDelay(1000/portTICK_PERIOD_MS);

    for(i=0; i<CIRCLE_1MM_LEN; i++)
    {
        a=pgm_read_float_near(&circle1mmangle1[i]);
        b=pgm_read_float_near(&circle1mmangle2[i]);
        stepper3angle=TRANSMISSION_RATIO_1*a*(-1);
        stepper4angle=TRANSMISSION_RATIO_2*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, NULL, 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, NULL, 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueSend(QueueHandle_Angle3, &stepper3angle, portMAX_DELAY);
        xQueueSend(QueueHandle_Angle4, &stepper4angle, portMAX_DELAY);

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, portMAX_DELAY);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, portMAX_DELAY);
        }

        //4 clear
        stepper3finishflag=false;
        stepper4finishflag=false;
        vTaskDelete(TaskHandle_Stepper3);
        vTaskDelete(TaskHandle_Stepper4);

        if(i==0)
        {
            servo2.attach(Pin_Servo2);
            vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
            ServoRotate(2, DOWN);
            vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
            Serial.println("Pen down!");
        }

        if(i==CIRCLE_1MM_LEN-1-1)
        {
            servo2.attach(Pin_Servo2);
            vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
            ServoRotate(2, UP);
            vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
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

    vTaskDelay(1000/portTICK_PERIOD_MS);

    for(i=0; i<LINE_LEN; i++)
    {
        a=pgm_read_float_near(&lineangle1[i]);
        b=pgm_read_float_near(&lineangle2[i]);
        stepper3angle=TRANSMISSION_RATIO_1*a*(-1);
        stepper4angle=TRANSMISSION_RATIO_2*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, NULL, 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, NULL, 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueSend(QueueHandle_Angle3, &stepper3angle, portMAX_DELAY);
        xQueueSend(QueueHandle_Angle4, &stepper4angle, portMAX_DELAY);

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, portMAX_DELAY);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, portMAX_DELAY);
        }

        //4 clear
        stepper3finishflag=false;
        stepper4finishflag=false;
        vTaskDelete(TaskHandle_Stepper3);
        vTaskDelete(TaskHandle_Stepper4);

        if(i==0)
        {
            servo2.attach(Pin_Servo2);
            vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
            ServoRotate(2, DOWN);
            vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
            Serial.println("Pen down!");
        }

        if(i==LINE_LEN-1-1)
        {
            servo2.attach(Pin_Servo2);
            vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
            ServoRotate(2, UP);
            vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
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

    vTaskDelay(1000/portTICK_PERIOD_MS);

    for(i=0; i<SQUARE_LEN; i++)
    {
        a=pgm_read_float_near(&squareangle1[i]);
        b=pgm_read_float_near(&squareangle2[i]);
        stepper3angle=TRANSMISSION_RATIO_1*a*(-1);
        stepper4angle=TRANSMISSION_RATIO_2*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, NULL, 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, NULL, 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueSend(QueueHandle_Angle3, &stepper3angle, portMAX_DELAY);
        xQueueSend(QueueHandle_Angle4, &stepper4angle, portMAX_DELAY);

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, portMAX_DELAY);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, portMAX_DELAY);
        }

        //4 clear
        stepper3finishflag=false;
        stepper4finishflag=false;
        vTaskDelete(TaskHandle_Stepper3);
        vTaskDelete(TaskHandle_Stepper4);

        if(i==0)
        {
            servo2.attach(Pin_Servo2);
            vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
            ServoRotate(2, DOWN);
            vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
            Serial.println("Pen down!");
        }

        if(i==SQUARE_LEN-1-1)
        {
            servo2.attach(Pin_Servo2);
            vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
            ServoRotate(2, UP);
            vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
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

    vTaskDelay(1000/portTICK_PERIOD_MS);

    //move to the first point
    for(i=0; i<1; i++)
    {
        a=pgm_read_float_near(&faceangle1[i]);
        b=pgm_read_float_near(&faceangle2[i]);
        stepper3angle=TRANSMISSION_RATIO_1*a*(-1);
        stepper4angle=TRANSMISSION_RATIO_2*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, NULL, 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, NULL, 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueSend(QueueHandle_Angle3, &stepper3angle, portMAX_DELAY);
        xQueueSend(QueueHandle_Angle4, &stepper4angle, portMAX_DELAY);

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, portMAX_DELAY);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, portMAX_DELAY);
        }

        //4 clear
        stepper3finishflag=false;
        stepper4finishflag=false;
        vTaskDelete(TaskHandle_Stepper3);
        vTaskDelete(TaskHandle_Stepper4);
    }
    PenDown();
    //draw face
    for(i=1; i<FACE_LEN; i++)
    {
        a=pgm_read_float_near(&faceangle1[i]);
        b=pgm_read_float_near(&faceangle2[i]);
        stepper3angle=TRANSMISSION_RATIO_1*a*(-1);
        stepper4angle=TRANSMISSION_RATIO_2*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, NULL, 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, NULL, 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueSend(QueueHandle_Angle3, &stepper3angle, portMAX_DELAY);
        xQueueSend(QueueHandle_Angle4, &stepper4angle, portMAX_DELAY);

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, portMAX_DELAY);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, portMAX_DELAY);
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
        a=pgm_read_float_near(&lefteyeangle1[i])-pgm_read_float_near(&faceangle1[i]);
        b=pgm_read_float_near(&lefteyeangle2[i])-pgm_read_float_near(&faceangle2[i]);
        stepper3angle=TRANSMISSION_RATIO_1*a*(-1);
        stepper4angle=TRANSMISSION_RATIO_2*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, NULL, 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, NULL, 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueSend(QueueHandle_Angle3, &stepper3angle, portMAX_DELAY);
        xQueueSend(QueueHandle_Angle4, &stepper4angle, portMAX_DELAY);

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, portMAX_DELAY);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, portMAX_DELAY);
        }

        //4 clear
        stepper3finishflag=false;
        stepper4finishflag=false;
        vTaskDelete(TaskHandle_Stepper3);
        vTaskDelete(TaskHandle_Stepper4);
    }
    PenDown();
    //draw left eye
    for(i=1; i<LEFTEYE_LEN; i++)
    {
        a=pgm_read_float_near(&lefteyeangle1[i]);
        b=pgm_read_float_near(&lefteyeangle2[i]);
        stepper3angle=TRANSMISSION_RATIO_1*a*(-1);
        stepper4angle=TRANSMISSION_RATIO_2*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, NULL, 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, NULL, 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueSend(QueueHandle_Angle3, &stepper3angle, portMAX_DELAY);
        xQueueSend(QueueHandle_Angle4, &stepper4angle, portMAX_DELAY);

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, portMAX_DELAY);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, portMAX_DELAY);
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
        a=pgm_read_float_near(&righteyeangle1[i])-pgm_read_float_near(&lefteyeangle1[i]);
        b=pgm_read_float_near(&righteyeangle2[i])-pgm_read_float_near(&lefteyeangle2[i]);
        stepper3angle=TRANSMISSION_RATIO_1*a*(-1);
        stepper4angle=TRANSMISSION_RATIO_2*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, NULL, 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, NULL, 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueSend(QueueHandle_Angle3, &stepper3angle, portMAX_DELAY);
        xQueueSend(QueueHandle_Angle4, &stepper4angle, portMAX_DELAY);

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, portMAX_DELAY);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, portMAX_DELAY);
        }

        //4 clear
        stepper3finishflag=false;
        stepper4finishflag=false;
        vTaskDelete(TaskHandle_Stepper3);
        vTaskDelete(TaskHandle_Stepper4);
    }
    PenDown();
    for(i=1; i<RIGHTEYE_LEN; i++)
    {
        a=pgm_read_float_near(&righteyeangle1[i]);
        b=pgm_read_float_near(&righteyeangle2[i]);
        stepper3angle=TRANSMISSION_RATIO_1*a*(-1);
        stepper4angle=TRANSMISSION_RATIO_2*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, NULL, 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, NULL, 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueSend(QueueHandle_Angle3, &stepper3angle, portMAX_DELAY);
        xQueueSend(QueueHandle_Angle4, &stepper4angle, portMAX_DELAY);

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, portMAX_DELAY);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, portMAX_DELAY);
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
        a=pgm_read_float_near(&mouthangle1[i])-pgm_read_float_near(&righteyeangle1[i]);
        b=pgm_read_float_near(&mouthangle2[i])-pgm_read_float_near(&righteyeangle2[i]);
        stepper3angle=TRANSMISSION_RATIO_1*a*(-1);
        stepper4angle=TRANSMISSION_RATIO_2*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, NULL, 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, NULL, 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueSend(QueueHandle_Angle3, &stepper3angle, portMAX_DELAY);
        xQueueSend(QueueHandle_Angle4, &stepper4angle, portMAX_DELAY);

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, portMAX_DELAY);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, portMAX_DELAY);
        }

        //4 clear
        stepper3finishflag=false;
        stepper4finishflag=false;
        vTaskDelete(TaskHandle_Stepper3);
        vTaskDelete(TaskHandle_Stepper4);
    }
    PenDown();
    for(i=1; i<MOUTH_LEN; i++)
    {
        a=pgm_read_float_near(&mouthangle1[i]);
        b=pgm_read_float_near(&mouthangle2[i]);
        stepper3angle=TRANSMISSION_RATIO_1*a*(-1);
        stepper4angle=TRANSMISSION_RATIO_2*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, NULL, 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, NULL, 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueSend(QueueHandle_Angle3, &stepper3angle, portMAX_DELAY);
        xQueueSend(QueueHandle_Angle4, &stepper4angle, portMAX_DELAY);

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, portMAX_DELAY);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, portMAX_DELAY);
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
        a=pgm_read_float_near(&mouthangle1[i])*(-1);
        b=pgm_read_float_near(&mouthangle2[i])*(-1);
        stepper3angle=TRANSMISSION_RATIO_1*a*(-1);
        stepper4angle=TRANSMISSION_RATIO_2*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, NULL, 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, NULL, 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueSend(QueueHandle_Angle3, &stepper3angle, portMAX_DELAY);
        xQueueSend(QueueHandle_Angle4, &stepper4angle, portMAX_DELAY);

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, portMAX_DELAY);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, portMAX_DELAY);
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

    vTaskDelay(1000/portTICK_PERIOD_MS);

    for(i=0; i<LOGO_LEN; i++)
    {
        a=pgm_read_float_near(&logoangle1[i]);
        b=pgm_read_float_near(&logoangle2[i]);
        stepper3angle=TRANSMISSION_RATIO_1*a*(-1);
        stepper4angle=TRANSMISSION_RATIO_2*b*(-1);

        //1 create task
        xTaskCreate(TaskStepper3, "Stepper3Rotate", 128, NULL, 1, &TaskHandle_Stepper3);
        xTaskCreate(TaskStepper4, "Stepper4Rotate", 128, NULL, 1, &TaskHandle_Stepper4);

        //2 transmit data
        xQueueSend(QueueHandle_Angle3, &stepper3angle, portMAX_DELAY);
        xQueueSend(QueueHandle_Angle4, &stepper4angle, portMAX_DELAY);

        //3 receive data
        while(stepper3finishflag==false || stepper4finishflag==false)
        {
            xQueueReceive(QueueHandle_Finish3, &stepper3finishflag, portMAX_DELAY);
            xQueueReceive(QueueHandle_Finish4, &stepper4finishflag, portMAX_DELAY);
        }

        //4 clear
        stepper3finishflag=false;
        stepper4finishflag=false;
        vTaskDelete(TaskHandle_Stepper3);
        vTaskDelete(TaskHandle_Stepper4);

        if(i==0)
        {
            servo2.attach(Pin_Servo2);
            vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
            ServoRotate(2, DOWN);
            vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
            Serial.println("Pen down!");
        }

        if(i==LOGO_LEN-1-1)
        {
            servo2.attach(Pin_Servo2);
            vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
            ServoRotate(2, UP);
            vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
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

    vTaskDelay(4000/portTICK_PERIOD_MS);
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
    pinMode(Pin_Magnet1, OUTPUT);

    digitalWrite(Pin_Magnet1, LOW);

    Serial.println("Magnet module has been initialized! ");
    delay(10);
}
void MagnetSwitch(uint8_t num, bool magnetswitch)
{
    if(num==1 && magnetswitch==ON)
    {
        digitalWrite(Pin_Magnet1, HIGH);
    }
    else if(num==1 && magnetswitch==OFF)
    {
        digitalWrite(Pin_Magnet1, LOW);
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
    if(num==1 && servostatus==UP)
    {
        for(int i=Servo1MinAngle; i<Servo1MaxAngle; i++)
        {
            servo1.write(i);
            vTaskDelay(16/portTICK_PERIOD_MS);
        }
    }
    else if(num==1 && servostatus==DOWN)
    {
        for(int i=Servo1MaxAngle; i>Servo1MinAngle; i--)
        {
            servo1.write(i);
            vTaskDelay(16/portTICK_PERIOD_MS);
        }
    }
    
    if(num==2 && servostatus==UP)
    {
        for(int i=Servo2MinAngle; i<Servo2MaxAngle; i++)
        {
            servo2.write(i);
            vTaskDelay(16/portTICK_PERIOD_MS);
        }
    }
    else if(num==2 && servostatus==DOWN)
    {
        for(int i=Servo2MaxAngle; i>Servo2MinAngle; i--)
        {
            servo2.write(i);
            vTaskDelay(16/portTICK_PERIOD_MS);
        }
    }
}
void PenDown(void)
{
    servo2.attach(Pin_Servo2);
    vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
    ServoRotate(2, DOWN);
    vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
    Serial.println("Pen down!");
}
void PenUp(void)
{
    servo2.attach(Pin_Servo2);
    vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
    ServoRotate(2, UP);
    vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
    Serial.println("Pen up!\n");
}
