#include "MQ4.h"

void MQ4::begin(){
    Ro = MQCalibration();
}

/*
 * Reads data from MQ4.
 *
 * Param data: the pointer to fill.
 */
void MQ4::read(MQ4_data_t *data){
   data->lpg = MQGetGasPercentage(MQRead()/Ro,GAS_LPG);
   data->h2 = MQGetGasPercentage(MQRead()/Ro,GAS_H2);
   data->ch4 = MQGetGasPercentage(MQRead()/Ro,GAS_CH4);
}

/*
 * reads data, returns LPG value in ppm
 */
float MQ4::readLPG(){
    return MQGetGasPercentage(MQRead()/Ro,GAS_LPG);
}

/*
 * reads data, returns H2 value in ppm
 */
float MQ4::readH2(){
    return MQGetGasPercentage(MQRead()/Ro,GAS_H2);
}

/*
 * reads data, returns CH4 value in ppm
 */
float MQ4::readCH4(){
    return MQGetGasPercentage(MQRead()/Ro,GAS_CH4);
}

/****************** MQResistanceCalculation ****************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/ 
float MQ4::MQResistanceCalculation(int raw_adc) {
   return (((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}

/***************************** MQCalibration ****************************************
Input:   mq_pin - analog channel
Output:  Ro of the sensor
Remarks: This function assumes that the sensor is in clean air. It use  
         MQResistanceCalculation to calculates the sensor resistance in clean air 
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about 
         10, which differs slightly between different sensors.
************************************************************************************/ 
float MQ4::MQCalibration() {                                                            // This should be done in 'clean air'
  float val=0;
  for (int i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {                                        //take multiple samples
    val += MQResistanceCalculation(_pin.read_u16()>>6);
    wait_ms(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                                                   //calculate the average value
  val = val/RO_CLEAN_AIR_FACTOR;                                                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro according to the chart in the datasheet 
  return val; 
}

/*****************************  MQRead *********************************************
Input:   mq_pin - analog channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/ 
float MQ4::MQRead() {
  int i;
  float rs=0;
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(_pin.read_u16()>>6);
    wait_ms(READ_SAMPLE_INTERVAL);
  }
  rs = rs/READ_SAMPLE_TIMES;
  return rs;  
}

/*****************************  MQGetGasPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the target gas
Remarks: This function passes different curves to the MQGetPercentage function which 
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/ 
float MQ4::MQGetGasPercentage(float rs_ro_ratio, GasType gas_id) {
    switch(gas_id) {
        case GAS_LPG: return MQGetPercentage(rs_ro_ratio,LPGCurve);
        case GAS_H2: return MQGetPercentage(rs_ro_ratio,H2Curve);
        case GAS_CH4: return MQGetPercentage(rs_ro_ratio,CH4Curve);
        default: return -1.0;
    } 
}

/*****************************  MQGetPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm) 
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a 
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic 
         value.
************************************************************************************/ 
int MQ4::MQGetPercentage(float rs_ro_ratio, float *pcurve) {
  return (pow(10,(((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}
// Return Ro calculated;
float MQ4::get_Ro() {
  return Ro;
}