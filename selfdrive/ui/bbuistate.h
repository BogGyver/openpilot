#pragma once
const int touch_timeout = 25;

typedef struct UICstmButton {
    char btn_name[6];
    char btn_label[6];
    char btn_label2[11];
} UICstmButton;

typedef struct BBUIState {
    float scr_scale_x;
    float scr_scale_y;
    int scr_w;
    int scr_h;
    float scr_device_factor;
    float scr_scissor_offset;
#if !defined(QCOM) && !defined(QCOM2)
    Display *scr_display;
#endif
    int touch_last_x;
    int touch_last_y;
    bool touch_last;
    int touch_timeout;
    int touch_last_width;
    bool shouldDrawFrame;
    UICstmButton btns[6];
    char btns_status[6];
    char car_model[40];
    char car_folder[20];
    Context *ctx;
    SubSocket *uiButtonInfo_sock;
    SubSocket *uiCustomAlert_sock;
    SubSocket *uiSetCar_sock;
    SubSocket *uiPlaySound_sock;
    PubSocket *uiButtonStatus_sock;
    SubSocket *gps_sock;
    SubSocket *uiGyroInfo_sock;
    Poller * poller;
    int btns_x[6];
    int btns_y[6];
    int btns_r[6];
    int custom_message_status;
    char custom_message[120];
    int img_logo;
    int img_logo2;
    int img_car;
    int tri_state_switch;
    long tri_state_switch_last_read;
    uint16_t maxCpuTemp;
    uint32_t maxBatTemp;
    float gpsAccuracy ;
    float freeSpace;
    float angleSteers;
    float angleSteersDes;
    float accPitch;
    float accRoll;
    float accYaw;
    float magPitch;
    float magRoll;
    float magYaw;
    float gyroPitch;
    float gyroRoll;
    float gyroYaw;
    bool icShowLogo;
    bool icShowCar;
    int batteryPercent;
    bool chargingEnabled;
    uint16_t fanSpeed;
    bool keepEonOff;
    bool recording;
} BBUIState;
