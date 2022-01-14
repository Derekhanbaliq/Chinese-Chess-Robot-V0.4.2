/*
 * Author       :  Derek Zhou
 * Date         :  06/15/2021
 * Version      :  Generation 4 Version 9 Pattern 2
 * New Function :  new pin config
 */

/********************************include*******************************/
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <FlexiTimer2.h>
#include <avr/pgmspace.h>
#include "anglematrix.h"

/********************************define********************************/
typedef struct
{
	int MotSpd;         //单位0.1°/s, 电机速度，也就是spdCur
	int PulseSpd;       //UNIT2PIT/|MotSpd| 一个pulse需要触发多少timer中断
    bool GpioLevel;
    int MotCur;         //记录电机当前位置 单位是脉冲数
}PulseMot_t; //电机控制参数结构体
typedef struct
{
	int PosCmd;         //0.1°
    int PosCur;         //0.1°
    int SpdCmd;         //0.1°/s
    int SpdCur;         //0.1°/s
    int AccCmd;         //0.1°/s2
    int AccIndevCmd;    //根据控制周期转化加速度 单位时间的速度增量
}Mot_t; //电机实际的运动参数结构体

#define DEBUG                   0       //debug状态开关 开启后省略繁琐的进程

#define OFF                     0
#define ON                      1
#define UP                      1
#define DOWN                    0
#define TRANSMISSION_RATIO_1    3.6
#define TRANSMISSION_RATIO_2    3
#define PULSE_PER_REVOLUTION    1600    //θ=1.8°, microstep=8, ∴360°÷θ×microstep=1600
#define CHESS_WAITING_TIME      100

//stepper control
#define MOTSPD_MIN              10      //单位0.1°/s, 电机最小速度1°/s
#define ARRIVAL_LIMIT           5       //单位0.1°, 行程<0.5°视为到位停止
#define UNIT2PIT                11250   //1000/0.1/(0.1/1.8*8*2) 主要用于计算 几次中断触发一次电平翻转
#define STEP2UNIT               1.125   //1.8/0.1/8/2 电平跳变一次走2.25(0.1°)/2的角度
#define TIMER_CLOCK             0.1     //0.1ms触发一次TimerCallback 这是最小时间单位
#define SPDPLAN_INTERVAL_MS     20      //20ms是控制周期的单位
#define READYPOS_MOT1           0       //Stepper1就绪位角度 起始位到就绪位后刷新
#define READYPOS_MOT2           0       //Stepper2就绪位角度 起始位到就绪位后刷新
#define SIGN(a)                 (abs(a)>0 ? (a)/abs(a) : 0) //速度方向符号
#define SPDCMD_C                2.4     //恒时速度系数
#define ACCCMD_C                1.2     //恒时加速度系数

/********************************extern********************************/
void Task_Main(void *pvParameters);
void Task_Stepper1(void *pvParameters);
void Task_Stepper2(void *pvParameters);
void Task_Stepper3(void *pvParameters);
void Task_Stepper4(void *pvParameters);
void Task_Mp3(void *pvParameters);

void SerialInit(void);
void RobotHandshake(String data);
void HandShakeJudge(String data, bool judgeflag);
void CheckInit(String initkeyword);
void Serial3ByteTranslate(char inchar);
void Serial0PrintRxData(String keyword);
void SendTxOK(String keyword);
void AnalyzeRxData(String keyword);
void CoordinateDataToArray(String keyword, String coordinatestring);
void Serial0PrintMoveOrEat(String keyword);
float CoordinateArrayToAngle(int x, int y, uint8_t steppernumber);
void Serial0PrintLastCoordinateArrayAddress(void);
void AnnounceResult(String result);
void AnnounceError(String error);
void CheckErrorStatus(bool errorflag);
void AnnounceFaceInfo(String faceinfo);
void ClearOut(void);

void LedInit(void);
void LedSwitch(uint8_t pin, bool status);

void ButtonInit(void);
void ButtonStartScan(void);
void ButtonGoScan(void);
void ButtonRegretScan(void);

void FlexiTimer2Init(void);
void TimerCallback(void);

void StepperInit(void);
void StepperStatusInit(void);
void SetPulseMot(int spd, PulseMot_t *pulsemot);
void SetSpdCmd(int spd, Mot_t* mot);
void SetAccCmd(int acc, Mot_t* mot);
void GetPulseMot(int *motspd, int *pulsespd, PulseMot_t *pulsemot);
void GetPosCur(PulseMot_t *pulsemot,Mot_t *mot);
void PlanOnce(Mot_t* mot);
bool ArrivalPoint(Mot_t* mot, PulseMot_t *pulsemot);
void Task_StepperAngleToPulseAndRotate(float angle, Mot_t *mot, PulseMot_t *pulsemot);
void RobotIsReady(void);
void RobotExecute(float stepper1angle, float stepper2angle);
void RobotMove(void);
void RobotEat(void);

void Mp3Init(void);
void Mp3ChangeVolume(char level);
void Task_Mp3Sounding(int index);
void Mp3Execute(int index);
void RandomInit(void);
void MP3RandomGoExecute(void);

void MagnetInit(void);
void MagnetSwitch(uint8_t num, bool magnetswitch);

void ServoInit(void);
void ServoRotate(uint8_t num, bool servostatus);
void ChessmanCatch(void);
void ChessmanSet(void);
void ChessmanDrop(void);

/********************************global********************************/
// NEW warning! 0 1 serial0, 2 interrupt, 9 10 Timer2, 13 44-46 unused PWM, 14 15 USART3
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
TaskControlBlock_t *TaskHandle_Stepper1, *TaskHandle_Stepper2, *TaskHandle_Stepper3, *TaskHandle_Stepper4;
TaskControlBlock_t *TaskHandle_Mp3;
QueueHandle_t  QueueHandle_Angle1, QueueHandle_Angle2, QueueHandle_Angle3, QueueHandle_Angle4; 
QueueHandle_t  QueueHandle_Finish1, QueueHandle_Finish2, QueueHandle_Finish3, QueueHandle_Finish4;
QueueHandle_t  QueueHandle_Mp3;
// init & start
bool BeginMainLoopFlag=false;
bool InitOkFlag=false;
bool FaceRecognitionFlag=false;
bool StartOkFlag=false;
bool GameStartFlag=false;
bool RobotWorkFlag=false;
// protocol
char InCharGlobal;
bool GetKeywordFlag=false;
String KeywordString="";
String CoordinateString="";
String ResultString="";
String ErrorData="";
String FaceInfoString="";
bool StringCompleteFlag=false;
uint8_t CoordinateCount=0; //count the coordinate char
uint8_t CoordinateData[8]={0};
float CurrentPosition[2]={0};
float TargetPosition[2]={0};
// angle matrix
extern const float Stepper1AngleMatrix[81][91] PROGMEM;
extern const float Stepper2AngleMatrix[81][91] PROGMEM;
// MP3
SoftwareSerial Mp3Serial(Pin_Mp3Tx, Pin_Mp3Rx); /*50--module TX, 51--module RX*/
// servo
Servo Servo1, Servo2;
const int Servo1MaxAngle=120;
const int Servo1MinAngle=75;
const int Servo2MaxAngle=110;
const int Servo2MinAngle=80;
// stepper control
PulseMot_t MotPulse_1, MotPulse_2, MotPulse_3, MotPulse_4;
Mot_t Mot_1, Mot_2, Mot_3, Mot_4;

/*******************************function*******************************/
void setup()
{
    SerialInit();
    LedInit();
    ButtonInit();
    StepperInit();
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
    QueueHandle_Mp3=xQueueCreate(1, sizeof(bool));
    Serial.println("Info queues have been created! ");

    xTaskCreate(Task_Main, "Main", 1024, NULL, 2, NULL); //can also be recognized as protocol+led+button task
    Serial.println("Tasks have been created! ");

#if DEBUG
    Serial0PrintLastCoordinateArrayAddress();
#endif
    Serial.println("Robot has been initialized!\n");
}
void loop()
{
    ;
}

// serveral tasks that need to be managed by FreeRTOS
void Task_Main(void *pvParameters)
{
    (void) pvParameters;
    
#if !DEBUG
    //initialization
    RobotHandshake("#initok\n"); //robot sends #initok and wait M20 sends #ok
    while(InitOkFlag==false) //robot wait M20 to send #initok and send #ok to answer
    {
        CheckInit("INIT");
    }

    //face recognition
    while(FaceRecognitionFlag==false)
    {
        CheckInit("FACE");
    }
#endif

    // robot is ready
    vTaskDelay(500/portTICK_PERIOD_MS);
    Mp3Execute(32); vTaskDelay(2000/portTICK_PERIOD_MS);
    RobotIsReady();
    StepperStatusInit(); //refresh the ready positions of motors
    Mp3Execute(31); 

#if !DEBUG
    //start
    while(GameStartFlag==false) //robot wait the player to press the start button
    {
        ButtonStartScan(); //contains CheckStartOk();
    }
#endif

    BeginMainLoopFlag=true;
    Serial.println("Get into the main loop! \n");

    //main loop
    while(1)
    {
        if(RobotWorkFlag==false) //counter disturbances of button while robot is working
        {
            ButtonGoScan();
            ButtonRegretScan();
        }

        while(Serial3.available() && StringCompleteFlag==false) //jump out as long as "true"
        {
            InCharGlobal=(char)Serial3.read();
            Serial3ByteTranslate(InCharGlobal);
            vTaskDelay(10/portTICK_PERIOD_MS); //wait the input char to be totally received into the buffer
        }
        if(StringCompleteFlag==true)
        {
            RobotWorkFlag=true;
            Serial0PrintRxData(KeywordString);
            SendTxOK(KeywordString);
            AnalyzeRxData(KeywordString);
            ClearOut();
            RobotWorkFlag=false;
        }

        vTaskDelay(10/portTICK_PERIOD_MS); // avoid task jam
    }
}
void Task_Stepper1(void *pvParameters)
{
    (void) pvParameters;

    float stepper1angle;
    bool stepper1finish=false;

    while(1)
    {
        if(xQueueReceive(QueueHandle_Angle1, &stepper1angle, portMAX_DELAY))
        {
            Task_StepperAngleToPulseAndRotate(stepper1angle, &Mot_1, &MotPulse_1);
        }
        stepper1finish=true;
        xQueueSend(QueueHandle_Finish1, &stepper1finish, portMAX_DELAY);
        stepper1angle=0;
        stepper1finish=false;
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}
void Task_Stepper2(void *pvParameters)
{
    (void) pvParameters;

    float stepper2angle;
    bool stepper2finish=false;

    while(1)
    {
        if(xQueueReceive(QueueHandle_Angle2, &stepper2angle, portMAX_DELAY))
        {
            Task_StepperAngleToPulseAndRotate(stepper2angle, &Mot_2, &MotPulse_2);
        }
        stepper2finish=true;
        xQueueSend(QueueHandle_Finish2, &stepper2finish, portMAX_DELAY);
        stepper2angle=0;
        stepper2finish=false;
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}
void Task_Stepper3(void *pvParameters)
{
    (void) pvParameters;

    float stepper3angle;
    bool stepper3finish=false;

    while(1)
    {
        if(xQueueReceive(QueueHandle_Angle3, &stepper3angle, portMAX_DELAY))
        {
            Task_StepperAngleToPulseAndRotate(stepper3angle, &Mot_3, &MotPulse_3);
        }
        stepper3finish=true;
        xQueueSend(QueueHandle_Finish3, &stepper3finish, portMAX_DELAY);
        stepper3angle=0;
        stepper3finish=false;
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}
void Task_Stepper4(void *pvParameters)
{
    (void) pvParameters;

    float stepper4angle;
    bool stepper4finish=false;

    while(1)
    {
        if(xQueueReceive(QueueHandle_Angle4, &stepper4angle, portMAX_DELAY))
        {
            Task_StepperAngleToPulseAndRotate(stepper4angle, &Mot_4, &MotPulse_4);
        }
        stepper4finish=true;
        xQueueSend(QueueHandle_Finish4, &stepper4finish, portMAX_DELAY);
        stepper4angle=0;
        stepper4finish=false;
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}
void Task_Mp3(void *pvParameters)
{
    (void) pvParameters;

    int mp3index;

    while(1)
    {
        if(xQueueReceive(QueueHandle_Mp3, &mp3index, portMAX_DELAY))
        {
            Task_Mp3Sounding(mp3index);
        }
        mp3index=0;
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

// Serial 0 & 3
void SerialInit(void)
{
    Serial.begin(115200);
    Serial3.begin(115200);
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
            // Serial.print("okResp="); Serial.println(okResp);
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
void CheckInit(String initkeyword)
{
    while(Serial3.available() && StringCompleteFlag==false) //jump out as long as "true"
    {
        InCharGlobal=(char)Serial3.read();
        Serial3ByteTranslate(InCharGlobal);
        vTaskDelay(10/portTICK_PERIOD_MS); //wait the input char to be totally received into the buffer
    }
    if(StringCompleteFlag==true)
    {
        Serial0PrintRxData(KeywordString);
        if(initkeyword=="INIT")
        {
            if(KeywordString=="initok" || KeywordString=="error")
            {
                SendTxOK(KeywordString);
                AnalyzeRxData(KeywordString);
                if(KeywordString=="initok")
                {
                    InitOkFlag=true;
                }
            }
            else
            {
                Serial.println("Sorry, only receive #initok right now~ ");
            }
        }
        else if(initkeyword=="FACE")
        {
            if(KeywordString=="reo" || KeywordString=="error")
            {
                SendTxOK(KeywordString);
                AnalyzeRxData(KeywordString);
                if(FaceInfoString=="stranger" || FaceInfoString=="@lb" || FaceInfoString=="@cys" || FaceInfoString=="@zmy" || FaceInfoString=="@tmy" || FaceInfoString=="@sh" || FaceInfoString=="@xl")
                {
                    FaceRecognitionFlag=true;
                }
            }
            else
            {
                Serial.println("Sorry, only receive #reo right now~ ");
            }
        }
        else if(initkeyword=="START")
        {
            if(KeywordString=="startok"|| KeywordString=="error")
            {
                SendTxOK(KeywordString);
                AnalyzeRxData(KeywordString);
                if(KeywordString=="startok")
                {
                    LedSwitch(Pin_LedGreen, ON);
                    StartOkFlag=true;
                }
            }
            else
            {
                Serial.println("Sorry, only receive #startok right now~ ");
            }
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
        GetKeywordFlag=true;
    }
    else if(inchar==58) // : keyword announcement finish
    {
        GetKeywordFlag=false;
    }
    else if(inchar==10) // \ keyword announcement & command finish \:92 \n:10 command: #initfinish no\n!!!!
    {
        GetKeywordFlag=false;
        StringCompleteFlag=true;
    }
    else if(GetKeywordFlag==true && (inchar>=97 && inchar<=122))
    {
        KeywordString+=inchar;
    }
    else if((KeywordString=="move" || KeywordString=="eat" || KeywordString=="movec" || KeywordString=="eatc" || KeywordString=="movew" ||KeywordString=="eatw") \
                                                                    && isDigit(inchar) && CoordinateCount<8) // get 4 data only
    {
        CoordinateString+=inchar;
        CoordinateCount++;
    }
    else if(KeywordString=="result" && (inchar>=97 && inchar<=122))
    {
        ResultString+=inchar;
    }
    else if(KeywordString=="error" && isDigit(inchar))
    {
        ErrorData+=inchar;
    }
    else if(KeywordString=="reo" && ((inchar>=97 && inchar<=122) || inchar==0x40)) //a-z @
    {
        FaceInfoString+=inchar;
    }
}
void Serial0PrintRxData(String keyword)
{
    Serial.print("\n");
    Serial.println("********************************");
    
    Serial.print("The keyword is: ");
    Serial.println(keyword);
    
    if(keyword=="move" || keyword=="movec" || keyword=="movew" || keyword=="eat" || keyword=="eatc" || keyword=="eatw")
    {
        Serial.print("The coordinate string is: ");
        if(CoordinateString=="")
        {
            Serial.println("NULL");
        }
        else
        {
            Serial.println(CoordinateString);
        }
    }
    if(keyword=="result")
    {
        Serial.print("The result is: ");
        Serial.println(ResultString);
    }
    if(keyword=="error")
    {
        Serial.print("The error is: ");
        Serial.println(ErrorData);
    }
    if(keyword=="reo")
    {
        Serial.print("The face-info-string is: ");
        Serial.println(FaceInfoString);
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
        // Serial.println("M20 says OK for this command. "); //no work to down
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
            AnnounceFaceInfo(FaceInfoString);
            vTaskDelay(4000/portTICK_PERIOD_MS);
        }
        else if(keyword=="startok")
        {
            Serial.println("M20 says startok~ ");
        }
        else if(keyword=="error")
        {
            Serial.println("!!!!!!!!!!!! ERROR !!!!!!!!!!!!! ");
            AnnounceError(ErrorData);
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
            CoordinateDataToArray(keyword, CoordinateString); //analyze the coordinate data
            Serial0PrintMoveOrEat(keyword);
            Mp3Execute(3);
            RobotMove(); // RobotMoveGranularity(); //to memorize the dean's method
            Mp3Execute(4);
            LedSwitch(Pin_LedGreen, ON);
        }
        else if(keyword=="eat")
        {
            Serial.println("EAT! ");
            CoordinateDataToArray(keyword, CoordinateString);
            Serial0PrintMoveOrEat(keyword);
            Mp3Execute(3);
            RobotEat();
            Mp3Execute(4);
            LedSwitch(Pin_LedGreen, ON);
        }
        else if(keyword=="movec")
        {
            Serial.println("CHECK! -MOVE-");
            CoordinateDataToArray(keyword, CoordinateString);
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
            CoordinateDataToArray(keyword, CoordinateString);
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
            CoordinateDataToArray(keyword, CoordinateString);
            Serial0PrintMoveOrEat(keyword);
            Mp3Execute(3);
            RobotMove();
            AnnounceResult("win"); //robot win player lose
        }
        else if(keyword=="eatw")
        {
            Serial.println("Checkmate! -EAT- ");
            CoordinateDataToArray(keyword, CoordinateString);
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
            AnnounceResult(ResultString);
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
            AnnounceError(ErrorData);
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
    for(int i=0; i<CoordinateCount; i++) //count<=7 is rational, which has been realized in the Serial3ByteTranslate()
    {
        CoordinateData[i]=int(coordinatestring[i]-48);
    }
    //create 2 coordinate arrays
    CurrentPosition[0]=CoordinateData[0]+0.1*CoordinateData[1];
    CurrentPosition[1]=CoordinateData[2]+0.1*CoordinateData[3];
    TargetPosition[0]=CoordinateData[4]+0.1*CoordinateData[5];
    TargetPosition[1]=CoordinateData[6]+0.1*CoordinateData[7];
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
    Serial.print(CurrentPosition[0], 1);
    Serial.print(", ");
    Serial.print(CurrentPosition[1], 1);
    Serial.print(")");
    Serial.print(" to (");
    Serial.print(TargetPosition[0], 1);
    Serial.print(", ");
    Serial.print(TargetPosition[1], 1);
    Serial.println(")... ");
}
float CoordinateArrayToAngle(int x, int y, uint8_t steppernumber)
{
    float stepper1angle, stepper2angle;

    stepper1angle=pgm_read_float_near(&Stepper1AngleMatrix[x][y]);
    stepper2angle=pgm_read_float_near(&Stepper2AngleMatrix[x][y]);

    if(steppernumber==1)
    {
        Serial.print("Stepper 1 angle: "); Serial.println(stepper1angle);
        return stepper1angle;
    }
    else if(steppernumber==2)
    {
        Serial.print("Stepper 2 angle: "); Serial.println(stepper2angle);
        return stepper2angle;
    }
}
void Serial0PrintLastCoordinateArrayAddress(void) //to check if the flash-reading progmem pointer is overflow
{
    uint16_t a, b;
    a=&Stepper1AngleMatrix[80][90]; //(81,91)
    b=&Stepper2AngleMatrix[80][90];
    Serial.println(a);
    Serial.println(b);
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
    KeywordString="";
    CoordinateString="";
    CoordinateCount=0;
    for(int j=0; j<CoordinateCount; j++) //clear out
    {
        CoordinateData[j]=0;
    }
    ResultString="";
    ErrorData="";
    FaceInfoString="";
    StringCompleteFlag=false;
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
                CheckInit("START");
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
    FlexiTimer2::set(1, TIMER_CLOCK/1000, TimerCallback); // FlexiTimer2::set(5, 1.0/10000, FlexiTimer2TogglePulse); // xtimes*0.00001sec T=basecnt*baseT 4 times fast
    FlexiTimer2::start(); //stepper rotating
}
void TimerCallback(void)
{
    static long mot_cnt_1 = 0;
    static long mot_cnt_2 = 0;
    static long mot_cnt_3 = 0;
    static long mot_cnt_4 = 0;
    int pulsespd, motspd;
    static bool level = true;

    GetPulseMot(&motspd, &pulsespd, &MotPulse_1);
    if(0 != motspd)
    {
        if(mot_cnt_1 > pulsespd)
        {
            digitalWrite(Pin_Stepper1Pulse, MotPulse_1.GpioLevel);
            MotPulse_1.GpioLevel = !MotPulse_1.GpioLevel;
            mot_cnt_1 = 0;
            if(motspd > 0)
            {
                MotPulse_1.MotCur++;
            }
            else
            {
                MotPulse_1.MotCur--;
            }
        }
        if(motspd > 0)
        {
            digitalWrite(Pin_Stepper1Direction, true);
        }
        else
        {
            digitalWrite(Pin_Stepper1Direction, false);
        }
        mot_cnt_1++;
    }

    GetPulseMot(&motspd, &pulsespd, &MotPulse_2);
    if(0 != motspd)
    {
        if(mot_cnt_2 > pulsespd)
        {
            digitalWrite(Pin_Stepper2Pulse, MotPulse_2.GpioLevel);
            MotPulse_2.GpioLevel = !MotPulse_2.GpioLevel;
            mot_cnt_2 = 0;
            if(motspd > 0)
            {
                MotPulse_2.MotCur++;
            }
            else
            {
                MotPulse_2.MotCur--;
            }
        }
        if(motspd > 0)
        {
            digitalWrite(Pin_Stepper2Direction, false);
        }
        else
        {
            digitalWrite(Pin_Stepper2Direction, true);
        }
        mot_cnt_2++;
    }

    GetPulseMot(&motspd, &pulsespd, &MotPulse_3);
    if(0 != motspd)
    {
        if(mot_cnt_3 > pulsespd)
        {
            digitalWrite(Pin_Stepper3Pulse, MotPulse_3.GpioLevel);
            MotPulse_3.GpioLevel = !MotPulse_3.GpioLevel;
            mot_cnt_3 = 0;
            if(motspd > 0)
            {
                MotPulse_3.MotCur++;
            }
            else
            {
                MotPulse_3.MotCur--;
            }
        }
        if(motspd > 0)
        {
            digitalWrite(Pin_Stepper3Direction, false);
        }
        else
        {
            digitalWrite(Pin_Stepper3Direction, true);
        }
        mot_cnt_3++;
    }

    GetPulseMot(&motspd, &pulsespd, &MotPulse_4);
    if(0 != motspd)
    {
        if(mot_cnt_4 > pulsespd)
        {
            digitalWrite(Pin_Stepper4Pulse, MotPulse_4.GpioLevel);
            MotPulse_4.GpioLevel = !MotPulse_4.GpioLevel;
            mot_cnt_4 = 0;
            if(motspd > 0)
            {
                MotPulse_4.MotCur++;
            }
            else
            {
                MotPulse_4.MotCur--;
            }
        }
        if(motspd > 0)
        {
            digitalWrite(Pin_Stepper4Direction, false);
        }
        else
        {
            digitalWrite(Pin_Stepper4Direction, true);
        }
        mot_cnt_4++;
    }
}

// stepper motor
void StepperInit(void)
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
    digitalWrite(Pin_Stepper4Enable, LOW); //work // digitalWrite(Pin_Stepper4Enable, HIGH); //wait to work

    StepperStatusInit();
}
void StepperStatusInit(void)
{
    memset(&Mot_1,0,sizeof(Mot_t));
    memset(&Mot_2,0,sizeof(Mot_t));
    memset(&Mot_3,0,sizeof(Mot_t));
    memset(&Mot_4,0,sizeof(Mot_t));
    memset(&MotPulse_1,0,sizeof(PulseMot_t));
    memset(&MotPulse_2,0,sizeof(PulseMot_t));
    memset(&MotPulse_3,0,sizeof(PulseMot_t));
    memset(&MotPulse_4,0,sizeof(PulseMot_t));

    SetSpdCmd(5000, &Mot_1);
    SetAccCmd(1000, &Mot_1);
    SetSpdCmd(5000, &Mot_2);
    SetAccCmd(1000, &Mot_2);
    SetSpdCmd(5000, &Mot_3);
    SetAccCmd(1000, &Mot_3);
    SetSpdCmd(5000, &Mot_4);
    SetAccCmd(1000, &Mot_4);
}
void SetPulseMot(int spd, PulseMot_t *pulsemot)
{
    taskENTER_CRITICAL(); //ISR互斥锁或者临界区保护  lock
    pulsemot->MotSpd = spd;
    if(spd!=0)
    {
        pulsemot->PulseSpd = UNIT2PIT/abs(spd);
        // Serial.print("PlanOnce PulseSpd "); Serial.println(pulsemot->PulseSpd);
    }
    taskEXIT_CRITICAL(); //ISR互斥锁或者临界区解锁  unlock
}
void SetSpdCmd(int spd, Mot_t* mot)
{
    mot->SpdCmd = spd;
}
void SetAccCmd(int acc, Mot_t* mot)
{
    mot->AccCmd = acc;
    mot->AccIndevCmd = mot->AccCmd*SPDPLAN_INTERVAL_MS/1000;
    if(mot->AccIndevCmd==0)
    {
        mot->AccIndevCmd=1; // Serial.print("SetAccCmd AccIndevCmd: "); Serial.println(mot->AccIndevCmd);
    }
}
void GetPulseMot(int *motspd, int *pulsespd, PulseMot_t *pulsemot)
{
    taskENTER_CRITICAL(); //ISR互斥锁或者临界区保护  lock
    *motspd = pulsemot->MotSpd;
    *pulsespd = pulsemot->PulseSpd;
    taskEXIT_CRITICAL(); //ISR互斥锁或者临界区解锁  unlock
}
void GetPosCur(Mot_t* mot, PulseMot_t *pulsemot)
{
    taskENTER_CRITICAL(); //ISR互斥锁或者临界区保护  lock
    mot->PosCur = STEP2UNIT*pulsemot->MotCur;
    taskEXIT_CRITICAL(); //ISR互斥锁或者临界区解锁  unlock
}
void PlanOnce(Mot_t* mot)
{
    int s,s_acc,s_dec,s_brake,spd,acc,accindev;
    int acctemp;

    s = mot->PosCmd - mot->PosCur;
    // Serial.print("PlanOnce PosCmd "); Serial.println(mot->PosCmd);
    // Serial.print("PlanOnce PosCur "); Serial.println(mot->PosCur);
    // Serial.print("PlanOnce s "); Serial.println(s);
    if(s>=0)
    {
        spd = abs(mot->SpdCmd);
        acc = abs(mot->AccCmd);
        accindev = abs(mot->AccIndevCmd);
    }
    else
    {
        spd = -abs(mot->SpdCmd);
        acc = -abs(mot->AccCmd);
        accindev = -abs(mot->AccIndevCmd);
    }
    // Serial.print("PlanOnce accindev "); Serial.println(accindev);
    // Serial.print("PlanOnce SpdCur "); Serial.println(mot->SpdCur);
    if(SIGN(spd) == SIGN(mot->SpdCur) && abs(mot->SpdCur) > abs(spd))
    {
        s_acc = -((long)spd * spd /acc/2 - (long)mot->SpdCur * mot->SpdCur /acc/2);    //减速
    }
    else
    {
        s_acc = (long)spd * spd /acc/2 - (long)mot->SpdCur * mot->SpdCur /acc/2;       //加速
    }
    // Serial.print("PlanOnce s_acc "); Serial.println(s_acc);
    s_dec = (long)spd * spd /acc/2;                                                    //减速
    // Serial.print("PlanOnce s_dec "); Serial.println(s_dec);
    if(abs(s) >= abs(s_acc+s_dec))
    {
        if(mot->SpdCur < (spd - abs(accindev)))
        {
            mot->SpdCur = mot->SpdCur + abs(accindev);
        }
        else if(mot->SpdCur > (spd+abs(accindev)))
        {
            mot->SpdCur = mot->SpdCur - abs(accindev);
        }
        else
        {
            mot->SpdCur = spd;
        }
    }
    else
    {
        s_brake = (long)mot->SpdCur * mot->SpdCur/acc/2;   //按目标减速度

        if(abs(s)<abs(s_brake) && SIGN(spd) == SIGN(mot->SpdCur))    //减速
        {
            acctemp = (long)mot->SpdCur*mot->SpdCur/s/2;
            mot->SpdCur = mot->SpdCur - acctemp*SPDPLAN_INTERVAL_MS/1000;
            if(abs(mot->SpdCur)<MOTSPD_MIN)
            {
                mot->SpdCur = SIGN(spd)*MOTSPD_MIN;                 //最低速运动
            }
        }
        else
        {
            mot->SpdCur = mot->SpdCur + accindev;                         //加速
        }
    }
    
}
bool ArrivalPoint(Mot_t* mot, PulseMot_t *pulsemot)
{
    bool arrival = false;
    int s;
    s = mot->PosCmd - mot->PosCur;
    // Serial.print("ArrivalPoint: "); Serial.println(s);
    if(abs(s) > ARRIVAL_LIMIT)
    {
        GetPosCur(mot,pulsemot);
        PlanOnce(mot);
    }
    else
    {
        arrival = true;
        mot->SpdCur = 0;
    }
    SetPulseMot(mot->SpdCur, pulsemot);
    return arrival;
}
// execute move & eat
void Task_StepperAngleToPulseAndRotate(float angle, Mot_t *mot, PulseMot_t *pulsemot)
{
    int poscmd = angle*10;
    int spdcmd;
    int acccmd;

    mot->PosCmd = poscmd;
    spdcmd = abs(mot->PosCmd - mot->PosCur)*SPDCMD_C;
    acccmd = abs(mot->PosCmd - mot->PosCur)*ACCCMD_C;
    SetSpdCmd(spdcmd, mot);
    SetAccCmd(acccmd, mot);
    
    while(!ArrivalPoint(mot, pulsemot))
    {   
        vTaskDelay(SPDPLAN_INTERVAL_MS/portTICK_PERIOD_MS);
    }
}
void RobotIsReady(void)
{
    float stepper1angle, stepper2angle, stepper3angle, stepper4angle;
    bool stepper1finishflag=false;
    bool stepper2finishflag=false;
    bool stepper3finishflag=false;
    bool stepper4finishflag=false;

    //1 create task
    stepper1angle=TRANSMISSION_RATIO_1*90;
    stepper2angle=TRANSMISSION_RATIO_2*90;
    stepper3angle=TRANSMISSION_RATIO_1*90;
    stepper4angle=TRANSMISSION_RATIO_2*(-90);
    xTaskCreate(Task_Stepper1, "Stepper1Task", 192, NULL, 1, &TaskHandle_Stepper1);
    xTaskCreate(Task_Stepper2, "Stepper2Task", 192, NULL, 1, &TaskHandle_Stepper2);
    xTaskCreate(Task_Stepper3, "Stepper3Task", 192, NULL, 1, &TaskHandle_Stepper3);
    xTaskCreate(Task_Stepper4, "Stepper4Task", 192, NULL, 1, &TaskHandle_Stepper4);
    vTaskDelay(10/portTICK_PERIOD_MS);
    //2 transmit data
    xQueueSend(QueueHandle_Angle1, &stepper1angle, portMAX_DELAY);
    xQueueSend(QueueHandle_Angle2, &stepper2angle, portMAX_DELAY);
    xQueueSend(QueueHandle_Angle3, &stepper3angle, portMAX_DELAY);
    xQueueSend(QueueHandle_Angle4, &stepper4angle, portMAX_DELAY);
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
void RobotExecute(float stepper1angle, float stepper2angle)
{
    bool stepper1finishflag=false;
    bool stepper2finishflag=false;

    //1 create task
    xTaskCreate(Task_Stepper1, "Stepper1Task", 192, NULL, 1, &TaskHandle_Stepper1);
    xTaskCreate(Task_Stepper2, "Stepper2Task", 192, NULL, 1, &TaskHandle_Stepper2);
    vTaskDelay(10/portTICK_PERIOD_MS);
    //2 transmit data
    xQueueSend(QueueHandle_Angle1, &stepper1angle, portMAX_DELAY);
    xQueueSend(QueueHandle_Angle2, &stepper2angle, portMAX_DELAY);
    vTaskDelay(10/portTICK_PERIOD_MS);
    //3 receive data
    while(stepper1finishflag==false || stepper2finishflag==false)
    {
        xQueueReceive(QueueHandle_Finish1, &stepper1finishflag, portMAX_DELAY);
        xQueueReceive(QueueHandle_Finish2, &stepper2finishflag, portMAX_DELAY);
    }
    vTaskDelay(10/portTICK_PERIOD_MS);
    //4 clear
    vTaskDelete(TaskHandle_Stepper1);
    vTaskDelete(TaskHandle_Stepper2);
    vTaskDelay(10/portTICK_PERIOD_MS);
}
void RobotMove(void)
{
    int x1=10*CurrentPosition[0];
    int y1=10*CurrentPosition[1];
    int x2=10*TargetPosition[0];
    int y2=10*TargetPosition[1];
    float stepper1angle1, stepper2angle1, stepper1angle2, stepper2angle2, return1, return2;

    Servo1.attach(Pin_Servo1);
    vTaskDelay(100/portTICK_PERIOD_MS);

    //to current coordinate
    stepper1angle1=TRANSMISSION_RATIO_1*CoordinateArrayToAngle(x1, y1, 1);
    stepper2angle1=TRANSMISSION_RATIO_2*CoordinateArrayToAngle(x1, y1, 2);
    RobotExecute(stepper1angle1, stepper2angle1);

    ChessmanCatch();

    //to target coordinate
    stepper1angle2=TRANSMISSION_RATIO_1*CoordinateArrayToAngle(x2, y2, 1); //delta1=stepper1angle2-stepper1angle1;
    stepper2angle2=TRANSMISSION_RATIO_2*CoordinateArrayToAngle(x2, y2, 2); //delta2=stepper2angle2-stepper2angle1;
    RobotExecute(stepper1angle2, stepper2angle2);

    ChessmanSet();

    //back to initial position
    return1=READYPOS_MOT1; //return1=(-1)*stepper1angle2;
    return2=READYPOS_MOT2; //return2=(-1)*stepper2angle2;
    RobotExecute(return1, return2);

    Servo1.detach();
    vTaskDelay(200/portTICK_PERIOD_MS);

    Serial.println("Move step is done. ");
}
void RobotEat(void)
{
    int x1=10*CurrentPosition[0];
    int y1=10*CurrentPosition[1];
    int x2=10*TargetPosition[0];
    int y2=10*TargetPosition[1];
    float stepper1angle1, stepper2angle1, stepper1angle2, stepper2angle2, return1, return2;
    bool stepper1finishflag=false;
    bool stepper2finishflag=false;

    Servo1.attach(Pin_Servo1);
    vTaskDelay(100/portTICK_PERIOD_MS);

    //to target coordinate
    stepper1angle2=TRANSMISSION_RATIO_1*CoordinateArrayToAngle(x2,y2,1);
    stepper2angle2=TRANSMISSION_RATIO_2*CoordinateArrayToAngle(x2,y2,2);
    RobotExecute(stepper1angle2, stepper2angle2);

    ChessmanCatch();

    //move the chess out (back to initial position)
    return1=READYPOS_MOT1; // return1=(-1)*stepper1angle2;
    return2=READYPOS_MOT2; // return2=(-1)*stepper2angle2;
    RobotExecute(return1, return2);

    ChessmanDrop();

    //to current coordinate
    stepper1angle1=TRANSMISSION_RATIO_1*CoordinateArrayToAngle(x1, y1, 1);
    stepper2angle1=TRANSMISSION_RATIO_2*CoordinateArrayToAngle(x1, y1, 2);
    RobotExecute(stepper1angle1, stepper2angle1);

    ChessmanCatch();

    //to target coordinate
    stepper1angle2=TRANSMISSION_RATIO_1*CoordinateArrayToAngle(x2, y2, 1); // delta1=stepper1angle2-stepper1angle1; delta1=stepper1angle2;
    stepper2angle2=TRANSMISSION_RATIO_2*CoordinateArrayToAngle(x2, y2, 2);
    RobotExecute(stepper1angle2, stepper2angle2);
    
    ChessmanSet();

    //back to initial position
    return1=READYPOS_MOT1; // return1=(-1)*stepper1angle2;
    return2=READYPOS_MOT2; // return2=(-1)*stepper2angle2;
    RobotExecute(return1, return2);
    Serial.println("Eat step is done. ");

    Servo1.detach();
    vTaskDelay(200/portTICK_PERIOD_MS);
}

// MP3
void Mp3Init(void)
{
    Mp3Serial.begin(9600);
    Mp3ChangeVolume(20);
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
void Task_Mp3Sounding(int index)
{
    int checkSum = 0;
    
    taskENTER_CRITICAL(); 
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
    taskEXIT_CRITICAL(); 

    vTaskDelay(4000/portTICK_PERIOD_MS);
}
void Mp3Execute(int index)
{
    xTaskCreate(Task_Mp3, "Mp3Task", 128, NULL, 2, &TaskHandle_Mp3);
    vTaskDelay(10/portTICK_PERIOD_MS);
    xQueueSend(QueueHandle_Mp3, &index, portMAX_DELAY);
    vTaskDelay(10/portTICK_PERIOD_MS);

    // vTaskDelay(4000/portTICK_PERIOD_MS);

    vTaskDelete(TaskHandle_Mp3);
    vTaskDelay(10/portTICK_PERIOD_MS);
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
    Servo1.attach(Pin_Servo1);
    Servo1.write(Servo1MaxAngle);
    Servo2.attach(Pin_Servo2);
    Servo2.write(Servo2MaxAngle);
    // delay(200);
    Servo1.detach();
    Servo2.detach();
    // delay(200);
}
void ServoRotate(uint8_t num, bool servostatus)
{
    if(num==1)
    {
        if(servostatus==UP)
        {
            for(int i=Servo1MinAngle; i<Servo1MaxAngle; i++)
            {
                Servo1.write(i);
                vTaskDelay(16/portTICK_PERIOD_MS);
            }
        }
        else if(servostatus==DOWN)
        {
            for(int i=Servo1MaxAngle; i>Servo1MinAngle; i--)
            {
                Servo1.write(i);
                vTaskDelay(16/portTICK_PERIOD_MS);
            }
        }
    }
    else if(num==2)
    {
        if(servostatus==UP)
        {
            for(int i=Servo2MinAngle; i<Servo2MaxAngle; i++)
            {
                Servo2.write(i);
                vTaskDelay(16/portTICK_PERIOD_MS);
            }
        }
        else if(servostatus==DOWN)
        {
            for(int i=Servo2MaxAngle; i>Servo2MinAngle; i--)
            {
                Servo2.write(i);
                vTaskDelay(16/portTICK_PERIOD_MS);
            }
        }
    }
}
void ChessmanCatch(void)
{
    vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
    MagnetSwitch(1, ON);
    ServoRotate(1, DOWN);
    vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
    ServoRotate(1, UP);
    vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
}
void ChessmanSet(void)
{
    vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
    ServoRotate(1, DOWN);
    MagnetSwitch(1, OFF);
    vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
    ServoRotate(1, UP);
    vTaskDelay(CHESS_WAITING_TIME/portTICK_PERIOD_MS);
}
void ChessmanDrop(void)
{
    vTaskDelay(CHESS_WAITING_TIME*2/portTICK_PERIOD_MS);
    MagnetSwitch(1, OFF);
    vTaskDelay(CHESS_WAITING_TIME*2/portTICK_PERIOD_MS);
}
