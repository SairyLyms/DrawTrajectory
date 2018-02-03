#include <ncurses.h>

union sI32ToByte
{
    int32_t integer;
    uint8_t byte[sizeof(integer)];
};

union sI16ToByte
{
    int16_t integer;
    uint8_t byte[sizeof(integer)];
};

union uI16ToByte
{
    uint16_t integer;
    uint8_t byte[sizeof(integer)];
};

void OpenBT(int *sockBT);
int ReadBT(int *sockBT);
void CloseBT(int *sockBT);
void DecodeVehicleData(uint8_t *readData,int8_t* stateMode,float* x,float* y,float* heading,float* yawAngle,float* yawRt,float* vel,float* odo);
void DecodeCourseData(uint8_t* readData,int* CourseID,float* xNext,float* yNext,float* headNext,float* phiV,float* phiU,float* h);
void SendCommandToVehicle(int8_t stateMode,int *sockBT);
void SetupNcurses(void);
int8_t VerifyCheckSum(uint8_t *arryToVerifyCheckSum,uint8_t nByte);

int sockBT;
int8_t stateMode;
float x,y,heading,yawAngle,yawRt,vel,odo,xNext,yNext,headNext,phiV,phiU,h;
int CourseID;

void OpenBT(int *sockBT)
{
    struct termios theTermios;
    memset(&theTermios, 0, sizeof(struct termios));
    cfmakeraw(&theTermios);
    cfsetspeed(&theTermios, 115200);

    theTermios.c_cflag = CREAD | CLOCAL;     // turn on READ
    theTermios.c_cflag |= CS8;
    theTermios.c_cc[VMIN] = 0;
    theTermios.c_cc[VTIME] = 10;     // 1 sec timeout
    ioctl(*sockBT, TIOCSETA, &theTermios);

    *sockBT = open("/dev/tty.RobotCAR-DevB", O_RDWR);

    if(*sockBT == -1)
    {
        std::cout << "Unable to open port" << std::endl;
    }

}

int ReadBT(int *sockBT)
{
    int i=0;
    uint8_t readState = 0;
    uint8_t readBytes = 255;
    uint8_t readData[255] = {};
    uint8_t vehicleDataRx[32] = {};
    std::string dispMessage;
    while(i < readBytes){
        int readCounter = read(*sockBT, &readData[i], 1);
        //ヘッダ読み込み時処理
        if(i>1 && readData[i-1] == 0xEC && readData[i] == 0xAB){
            memset(readData,0,sizeof(readData));
            readCounter = 0;
            i = 0;
            readState = 1;  //ヘッダ読み込み完了
        }
        //フッタ読み込み時処理
        else if(readState == 1 && i>1 && readData[i-1] == 0xED && readData[i] == 0xDA){
            //チェックサム検証、OKでデータ読み出し
            if(VerifyCheckSum(readData,32)){
                memcpy(vehicleDataRx,readData,32);
                readState = 2;
                break;
            }
        }
        i += readCounter;
    }
    switch(readData[0]){
        case 1:DecodeVehicleData(readData,&stateMode,&x,&y,&heading,&yawAngle,&yawRt,&vel,&odo);break;   //車両データのデコード
        case 2:DecodeCourseData(readData,&CourseID,&xNext,&yNext,&headNext,&phiV,&phiU,&h);break; //コースデータのデコード
        default:break;
    }
    move(0,0);
    dispMessage =   "Mode," + std::to_string((int)stateMode) + ",x," + std::to_string(x) + ",y," + std::to_string(y);
    dispMessage +=  ",head," + std::to_string(heading) + ",YAn," + std::to_string(yawAngle) + ",YRt," + std::to_string(yawRt);
    printw("%s",dispMessage.c_str());
    move(1,0);
    dispMessage =  ",Vel," + std::to_string(vel) + ",Odo," + std::to_string(odo);
    printw("%s",dispMessage.c_str());
    dispMessage =   "CourseID," + std::to_string(CourseID) + ",xNext," + std::to_string(xNext) + ",yNext," + std::to_string(yNext);
    dispMessage +=  "headNext," + std::to_string(headNext) + ",phiV," + std::to_string(phiV) + ",phiU," + std::to_string(phiU);
    move(2,0);
    printw("%s",dispMessage.c_str());
    return 0;
}

void CloseBT(int *sockBT)
{
    close(*sockBT);    
}


int8_t VerifyCheckSum(uint8_t *arryToVerifyCheckSum,uint8_t nByte)
{
    uint8_t checkSum = 0;
    for(uint8_t i=0;i<(nByte-1);i++){
        checkSum += *(arryToVerifyCheckSum + i);
    }
    if(checkSum == *(arryToVerifyCheckSum + nByte-1)){return 1;}
    else{return 0;}
}

void DecodeVehicleData(uint8_t* readData,int8_t* stateMode,float* x,float* y,float* heading,float* yawAngle,float* yawRt,float* vel,float* odo)
{
    union sI32ToByte I32Bx,I32By;
    union sI16ToByte I16Bheading,I16ByawAngle,I16ByawRt;
    union uI16ToByte uI16Bvel,uI16Bodo;
    *stateMode = *(readData + 1);
    memcpy(I32Bx.byte,&readData[2],4);*x = float(I32Bx.integer) * 0.01;
    memcpy(I32By.byte,&readData[6],4);*y = float(I32By.integer) * 0.01;
    memcpy(I16Bheading.byte,&readData[10],2);*heading = float(I16Bheading.integer) * 0.0001;
    memcpy(I16ByawAngle.byte,&readData[12],2);*yawAngle = float(I16ByawAngle.integer) * 0.0001;
    memcpy(I16ByawRt.byte,&readData[14],2);*yawRt = float(I16ByawRt.integer) * 0.001;
    memcpy(uI16Bvel.byte,&readData[16],2);*vel = float(uI16Bvel.integer) * 0.01;
    memcpy(uI16Bodo.byte,&readData[18],2);*odo = float(uI16Bodo.integer) * 0.01;
}

void DecodeCourseData(uint8_t* readData,int* courseID,float* xNext,float* yNext,float* headNext,float* phiV,float* phiU,float* h)
{
    union sI32ToByte I32BxNext,I32ByNext;
    union sI16ToByte I16BheadNext,I16BphiV,I16BphiU,I16BcourseID;
    union uI16ToByte uI16Bh;
    static int lastCourseID = 0;
    memcpy(I16BcourseID.byte,&readData[2],2);*courseID = int16_t(I16BcourseID.integer);
    if(*courseID != lastCourseID){
        memcpy(I32BxNext.byte,&readData[4],4);*xNext = float(I32BxNext.integer) * 0.01;
        memcpy(I32ByNext.byte,&readData[8],4);*yNext = float(I32ByNext.integer) * 0.01;
        memcpy(I16BheadNext.byte,&readData[12],2);*headNext = float(I16BheadNext.integer) * 0.0001;
        memcpy(I16BphiV.byte,&readData[14],2);*phiV = float(I16BphiV.integer) * 0.001;
        memcpy(I16BphiU.byte,&readData[16],2);*phiU = float(I16BphiU.integer) * 0.001;
        memcpy(uI16Bh.byte,&readData[18],2);*h = float(uI16Bh.integer) * 0.01;
    }
    lastCourseID = *courseID;
}

void SendCommandToVehicle(int8_t stateMode,int *sockBT)
{
    std::string dispMessage;
    uint8_t key = 0;
    //車両状態表示,入力要求表示
    switch(stateMode){
        case 0x01 : dispMessage = "press [c] key to calib."; break;
        case 0x03 : dispMessage = "prepare to calib."; break;
        case 0x05 : dispMessage = "calib..."; break;
        case 0x09 : dispMessage = "press [r] key to start."; break;
        case 0x19 : dispMessage = "press ANY key to stop."; break;
        default   : dispMessage = "stop."; break;
    }
    move(3,0);
    printw("%s",dispMessage.c_str());
    key = getch();
    if(key){write(*sockBT, &key, 1);};
}

void SetupNcurses(void)
{
    initscr();
    cbreak();
    noecho();
    //scrollok(stdscr, TRUE);
    nodelay(stdscr, TRUE);
}