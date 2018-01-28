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

int OpenBT(int *sockBT);
int ReadBT(int *sockBT);
void CloseBT(int *sockBT);
void DecodeVehicleData(uint8_t *readData,int8_t* stateMode,float* x,float* y,float* heading,float* yawAngle,float* yawRt,float* vel);
int8_t VerifyCheckSum(uint8_t *arryToVerifyCheckSum,uint8_t nByte);

int sockBT;
float x,y,heading,yawAngle,yawRt,vel;


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

    uint8_t vehicleDataRx[32] = {};

    int8_t stateMode;
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
        case 1:DecodeVehicleData(readData,&stateMode,&x,&y,&heading,&yawAngle,&yawRt,&vel);break;   //車両データのデコード
        case 2:    std::cout << "Course" << std::endl;break; //コースデータのデコード処理入れる
        default:break;
    }
    std::cout<< "Mode,"<<stateMode << ",x," <<x<< ",y," <<y<< ",head," <<heading<< ",YAn," <<yawAngle<< ",YRt," <<yawRt<< ",Vel," <<vel<<std::endl;
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

void DecodeVehicleData(uint8_t *readData,int8_t* stateMode,float* x,float* y,float* heading,float* yawAngle,float* yawRt,float* vel)
{
    union sI32ToByte I32Bx,I32By;
    union sI16ToByte I16Bheading,I16ByawAngle,I16ByawRt;
    union uI16ToByte uI16Bvel;
    *stateMode = *(readData + 1);
    memcpy(I32Bx.byte,&readData[2],4);*x = float(I32Bx.integer) * 0.01;
    memcpy(I32By.byte,&readData[6],4);*y = float(I32By.integer) * 0.01;
    memcpy(I16Bheading.byte,&readData[10],2);*heading = float(I16Bheading.integer) * 0.0001;
    memcpy(I16ByawAngle.byte,&readData[12],2);*yawAngle = float(I16ByawAngle.integer) * 0.0001;
    memcpy(I16ByawRt.byte,&readData[14],2);*yawRt = float(I16ByawRt.integer) * 0.001;
    memcpy(uI16Bvel.byte,&readData[16],2);*vel = float(uI16Bvel.integer) * 0.01;
}