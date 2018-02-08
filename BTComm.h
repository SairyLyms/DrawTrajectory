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

int Serialport_init(void);
int ReadBT(int *sockBT);
void CloseBT(int *sockBT);
void DecodeVehicleData(uint8_t *readData,int8_t* stateMode,float* x,float* y,float* heading,float* yawAngle,float* yawRt,float* vel,float* odo);
void DecodeCourseData(uint8_t* readData,int16_t* courseID,float* xNext,float* yNext,float* headNext,float* phiV,float* phiU,float* h);
void DecodeData(uint8_t* readData,int8_t* stateMode,int16_t* courseID,float* x,float* y,float* heading,float* yawAngle,float* yawRt,float* vel,float* odo,float* xNext,float* yNext,float* headNext,float* phiV,float* phiU,float* h);
void SendCommandToVehicle(int8_t stateMode,int *sockBT);
void SetupNcurses(void);
int8_t VerifyCheckSum(uint8_t *arryToVerifyCheckSum,uint8_t nByte);

int8_t stateMode;
int16_t courseID;
float x,y,heading,yawAngle,yawRt,vel,odo,xNext,yNext,headNext,phiV,phiU,h;

int Serialport_init(void)
{
    const int baud = 115200;
    const char serialport[] = "/dev/ttyUSB0";
    struct termios toptions;
    int fd;
    
    //fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
    fd = open(serialport, O_RDWR | O_NONBLOCK );
    
    if (fd == -1)  {
        perror("serialport_init: Unable to open port ");
        return -1;
    }
    
    //int iflags = TIOCM_DTR;
    //ioctl(fd, TIOCMBIS, &iflags);     // turn on DTR
    //ioctl(fd, TIOCMBIC, &iflags);    // turn off DTR

    if (tcgetattr(fd, &toptions) < 0) {
        perror("serialport_init: Couldn't get term attributes");
        return -1;
    }
    speed_t brate = baud; // let you override switch below if needed
    switch(baud) {
    case 4800:   brate=B4800;   break;
    case 9600:   brate=B9600;   break;
#ifdef B14400
    case 14400:  brate=B14400;  break;
#endif
    case 19200:  brate=B19200;  break;
#ifdef B28800
    case 28800:  brate=B28800;  break;
#endif
    case 38400:  brate=B38400;  break;
    case 57600:  brate=B57600;  break;
    case 115200: brate=B115200; break;
    }
    cfsetispeed(&toptions, brate);
    cfsetospeed(&toptions, brate);

    // 8N1
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    // no flow control
    toptions.c_cflag &= ~CRTSCTS;

    //toptions.c_cflag &= ~HUPCL; // disable hang-up-on-close to avoid reset

    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw

    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 0;
    //toptions.c_cc[VTIME] = 20;
    
    tcsetattr(fd, TCSANOW, &toptions);
    if( tcsetattr(fd, TCSAFLUSH, &toptions) < 0) {
        perror("init_serialport: Couldn't set term attributes");
        return -1;
    }

    return fd;
}


int OpenBT(int *sockBT)
{
    struct sockaddr_rc addr = { 0 };
    int status;
    char dest[18] = "20:15:12:29:14:76";

    // allocate a socket
    *sockBT = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    // set the connection parameters (who to connect to)
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t) 1;
    str2ba( dest, &addr.rc_bdaddr );

    // connect to server
    status = connect(*sockBT, (struct sockaddr *)&addr, sizeof(addr));
    if( status < 0 ) perror("not establish the connection.");

    return status;
}

int ReadBT(int *sockBT)
{
    int i=0;
    uint8_t readState = 0;
    uint8_t readBytes = 255;
    uint8_t readData[255] = {};
    uint8_t vehicleDataRx[64] = {};
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
            if(VerifyCheckSum(readData,64)){
                memcpy(vehicleDataRx,readData,64);
                readState = 2;
                break;
            }
        }
        i += readCounter;
    }
    if(readState == 2){
        switch(readData[0]){
            case 0:DecodeData(readData,&stateMode,&courseID,&x,&y,&heading,&yawAngle,&yawRt,&vel,&odo,&xNext,&yNext,&headNext,&phiV,&phiU,&h);break;
            //case 1:DecodeVehicleData(readData,&stateMode,&x,&y,&heading,&yawAngle,&yawRt,&vel,&odo);break;   //車両データのデコード
            //case 2:DecodeCourseData(readData,&courseID,&xNext,&yNext,&headNext,&phiV,&phiU,&h);break; //コースデータのデコード
            default:break;
        }
    }
    move(0,0);
    dispMessage =   "Mode," + std::to_string((int)stateMode) + ",x," + std::to_string(x) + ",y," + std::to_string(y);
    dispMessage +=  ",head," + std::to_string(heading) + ",YAn," + std::to_string(yawAngle) + ",YRt," + std::to_string(yawRt);
    printw("%s",dispMessage.c_str());
    move(1,0);
    dispMessage =  ",Vel," + std::to_string(vel) + ",Odo," + std::to_string(odo);
    printw("%s",dispMessage.c_str());
    dispMessage =   "courseID," + std::to_string(courseID) + ",xNext," + std::to_string(xNext) + ",yNext," + std::to_string(yNext);
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

void DecodeCourseData(uint8_t* readData,int16_t* courseID,float* xNext,float* yNext,float* headNext,float* phiV,float* phiU,float* h)
{
    union sI32ToByte I32BxNext,I32ByNext;
    union sI16ToByte I16BheadNext,I16BphiV,I16BphiU,I16BcourseID;
    union uI16ToByte uI16Bh;
    static int lastcourseID = 0;
    memcpy(I16BcourseID.byte,&readData[2],2);*courseID = int16_t(I16BcourseID.integer);
    if(*courseID != lastcourseID){
        memcpy(I32BxNext.byte,&readData[4],4);*xNext = float(I32BxNext.integer) * 0.01;
        memcpy(I32ByNext.byte,&readData[8],4);*yNext = float(I32ByNext.integer) * 0.01;
        memcpy(I16BheadNext.byte,&readData[12],2);*headNext = float(I16BheadNext.integer) * 0.0001;
        memcpy(I16BphiV.byte,&readData[14],2);*phiV = float(I16BphiV.integer) * 0.001;
        memcpy(I16BphiU.byte,&readData[16],2);*phiU = float(I16BphiU.integer) * 0.001;
        memcpy(uI16Bh.byte,&readData[18],2);*h = float(uI16Bh.integer) * 0.01;
    }
    lastcourseID = *courseID;
}

void DecodeData(uint8_t* readData,int8_t* stateMode,int16_t* courseID,float* x,float* y,float* heading,float* yawAngle,float* yawRt,float* vel,float* odo,float* xNext,float* yNext,float* headNext,float* phiV,float* phiU,float* h)
{
    union sI32ToByte I32Bx,I32By,I32BxNext,I32ByNext;
    union sI16ToByte I16Bheading,I16ByawAngle,I16ByawRt,I16BheadNext,I16BphiV,I16BphiU,I16BcourseID;
    union uI16ToByte uI16Bvel,uI16Bodo,uI16Bh;
    *stateMode = *(readData + 1);
    memcpy(I16BcourseID.byte,&readData[2],2);*courseID = int16_t(I16BcourseID.integer);
    memcpy(I32Bx.byte,&readData[4],4);*x = float(I32Bx.integer) * 0.01;
    memcpy(I32By.byte,&readData[8],4);*y = float(I32By.integer) * 0.01;
    memcpy(I16Bheading.byte,&readData[12],2);*heading = float(I16Bheading.integer) * 0.0001;
    memcpy(I16ByawAngle.byte,&readData[14],2);*yawAngle = float(I16ByawAngle.integer) * 0.0001;
    memcpy(I16ByawRt.byte,&readData[16],2);*yawRt = float(I16ByawRt.integer) * 0.001;
    memcpy(uI16Bvel.byte,&readData[18],2);*vel = float(uI16Bvel.integer) * 0.01;
    memcpy(uI16Bodo.byte,&readData[20],2);*odo = float(uI16Bodo.integer) * 0.01;
    memcpy(I32BxNext.byte,&readData[22],4);*xNext = float(I32BxNext.integer) * 0.01;
    memcpy(I32ByNext.byte,&readData[26],4);*yNext = float(I32ByNext.integer) * 0.01;
    memcpy(I16BheadNext.byte,&readData[30],2);*headNext = float(I16BheadNext.integer) * 0.0001;
    memcpy(I16BphiV.byte,&readData[32],2);*phiV = float(I16BphiV.integer) * 0.001;
    memcpy(I16BphiU.byte,&readData[34],2);*phiU = float(I16BphiU.integer) * 0.001;
    memcpy(uI16Bh.byte,&readData[36],2);*h = float(uI16Bh.integer) * 0.01;
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