#ifndef MQ4_h
#define MQ4_h

#include "mbed.h"

#define RL_VALUE                    10                                           //define the load resistance on the board, in kilo ohms
#define RO_DEFAULT                  10                                           //Ro is initialized to 10 kilo ohms
#define RO_CLEAN_AIR_FACTOR         4.4f                                        //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO, which is derived from the chart in datasheet
#define CALIBARAION_SAMPLE_TIMES    5                                           //define how many samples you are going to take in the calibration phase
#define CALIBRATION_SAMPLE_INTERVAL 50                                          //define the time interal(in milisecond) between each samples
#define READ_SAMPLE_INTERVAL        50                                          //define how many samples you are going to take in normal operation
#define READ_SAMPLE_TIMES           5                                           //define the time interal(in milisecond) between each samples

//The curves
// {log(X1) ,log(Y),[log(Y1) - log(Y2)] / [log(X2) - log(X1)]} get values from datasheet             
static float LPGCurve[]   = {2.3f,0.43f,-0.33f};
static float H2Curve[]    = {2.3f,0.57f,-0.16f};
static float CH4Curve[]   = {2.3f,0.27f,-0.38f};

typedef struct {
  float lpg;
  float h2;
  float ch4;
} MQ4_data_t;

typedef enum  {
    GAS_LPG = 0,
    GAS_H2 = 1,
    GAS_CH4 = 2
} GasType;


class MQ4 {
public: 
    MQ4(PinName pin) : _pin(pin){
      Ro = RO_DEFAULT;
    };
    void read(MQ4_data_t *ptr);
    float readLPG();
    float readH2();
    float readCH4();
    float get_Ro();
    void begin();
private:
    AnalogIn _pin;
    float MQRead();
    float MQGetGasPercentage(float rs_ro_ratio, GasType gas_id);
    int MQGetPercentage(float rs_ro_ratio, float *pcurve);
    float MQCalibration();
    float MQResistanceCalculation(int raw_adc);
    float Ro;
    
};

#endif