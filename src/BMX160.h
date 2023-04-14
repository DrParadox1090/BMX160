#ifndef BMX160_H
#define BMX160_H

#include <Arduino.h>
#include "i2cdevice.h"
#include "BMX160_REG.h"

class bmx160 : protected i2cdevice{
    private : 
        byte s_address = 0x68;
        float_t acc[3] {0.0 , 0.0 , 0.0};
        float_t gyro[3] {0.0 , 0.0 , 0.0};
        float_t mag[3] {0.0 , 0.0 , 0.0};

        float_t ACC_SENSITIVITY , GYRO_SENSITIVITY ;
        float_t Acc_range , Gyro_range , Mag_range[2];
        byte read_acc_powermode();
        byte read_gyro_powermode();
        byte read_mag_if_powermode();
        void acc_normal_mode();
        void gyro_normal_mode();
        void mag_normal_mode();
        void softreset();

        void sensor_PMU_status(int ch) ;
        void acc_PMU();

        void set_acc_range(byte RANGE);
        void set_gyro_range(byte RANGE);
        void setup_mag_if();

    public :
        bmx160(TwoWire *p);

        void get_acc_data();
        void get_gyro_data();
        void get_mag_data();
    
};




//Initialize sensor and set all sensors to NORMAL MODE
bmx160::bmx160(TwoWire *p){
    setupDevice(s_address , p);
    if(checkConnection()){
        Mag_range[0] = 2300;
        Mag_range[1] = 5000;
        softreset();
        acc_normal_mode();
        gyro_normal_mode();
        mag_normal_mode();
        setup_mag_if();
        set_acc_range(ACCEL_RANGE_16G);
        set_gyro_range(GYRO_RANGE_125DPS);
    }
}

void bmx160::set_acc_range(byte RANGE){
    writeByteToReg(ACC_RANGE , RANGE);
    switch(RANGE){
        case (ACCEL_RANGE_2G):{
            ACC_SENSITIVITY = ACC_SENSITIVITY_2G;
            Acc_range = 4.0F;
            break;
        }
        case (ACCEL_RANGE_4G):{
            ACC_SENSITIVITY = ACC_SENSITIVITY_4G;
            Acc_range = 8.0F;
            break;
        }
        case (ACCEL_RANGE_8G):{
            ACC_SENSITIVITY = ACC_SENSITIVITY_8G;
            Acc_range = 16.0F;
            break;
        }
        case (ACCEL_RANGE_16G):{
            ACC_SENSITIVITY = ACC_SENSITIVITY_16G;
            Acc_range = 32.0F;
            break;
        }
    }
}

void bmx160::set_gyro_range(byte RANGE){
    writeByteToReg(GYRO_RANGE , RANGE);
    switch(RANGE){
        case(GYRO_RANGE_125DPS):{
            GYRO_SENSITIVITY = GYRO_SENSITIVITY_125DPS;
            Gyro_range = 250.0F;
            break;
        }
        case(GYRO_RANGE_250DPS):{
            GYRO_SENSITIVITY = GYRO_SENSITIVITY_250DPS;
            Gyro_range = 500.0F;
            break;
        }
        case(GYRO_RANGE_500DPS):{
            GYRO_SENSITIVITY = GYRO_SENSITIVITY_500DPS;
            Gyro_range = 1000.0F;
            break;
        }
        case(GYRO_RANGE_1000DPS):{
            GYRO_SENSITIVITY = GYRO_SENSITIVITY_1000DPS;
            Gyro_range = 2000.0F;
            break;
        }
        case(GYRO_RANGE_2000DPS):{
            GYRO_SENSITIVITY = GYRO_SENSITIVITY_2000DPS;
            Gyro_range = 4000.0F;
            break;
        }
    }
}

void bmx160::setup_mag_if(){
    writeByteToReg(MAG_IF0 , 0x80);
    delay(50);
    writeByteToReg(MAG_IF3 , 0x01);
    writeByteToReg(MAG_IF2 , 0x4B);

    writeByteToReg(MAG_IF3 , 0x04);
    writeByteToReg(MAG_IF2 , 0x51);

    writeByteToReg(MAG_IF3 , 0x0E);
    writeByteToReg(MAG_IF2 , 0x52);

    writeByteToReg(MAG_IF3, 0x02);
    writeByteToReg(MAG_IF2, 0x4C);
    writeByteToReg(MAG_IF1, 0x42);
    writeByteToReg(MAG_CONF, 0x08);
    writeByteToReg(MAG_IF0, 0x03);
}

void bmx160::acc_normal_mode(){
    writeByteToReg(CMD , 0x11);
    delay(50);

}

void bmx160::gyro_normal_mode(){
    writeByteToReg(CMD , 0x15);
    delay(100);
}

void bmx160::mag_normal_mode(){
    writeByteToReg(CMD , 0x19);
    delay(10);
}

void bmx160::softreset(){
    writeByteToReg(CMD , SOFT_RESET);
    delay(15);
}

void bmx160::acc_PMU(){
    writeByteToReg(0x7E , ACC_SET_PMU_MODE&NORMAL);
    delay(6.8);
}

void bmx160::get_acc_data(){
    acc[0] = ((readByteFromReg(0x13) << 8) | (readByteFromReg(0x12))) * ACC_SENSITIVITY;
    acc[1] = ((readByteFromReg(0x15) << 8) | (readByteFromReg(0x14))) * ACC_SENSITIVITY;
    acc[2] = ((readByteFromReg(0x17) << 8) | (readByteFromReg(0x16))) * ACC_SENSITIVITY;
    for(int i = 0 ; i < 3 ; i++){ 
        if(acc[i] > (Acc_range/2.0)){
            acc[i] = acc[i] - Acc_range;
        }
    }
    
    Serial.print("Acc X : ");
    Serial.print(acc[0]); Serial.print("\t");
    Serial.print("Acc Y : ");
    Serial.print(acc[1]); Serial.print("\t");
    Serial.print("Acc Z : ");
    Serial.println(acc[2]);
}

void bmx160::get_gyro_data(){
    gyro[0] = ((readByteFromReg(0x0D) << 8) | (readByteFromReg(0x0C))) * GYRO_SENSITIVITY; 
    gyro[0] = ((readByteFromReg(0x0F) << 8) | (readByteFromReg(0x0E))) * GYRO_SENSITIVITY; 
    gyro[0] = ((readByteFromReg(0x11) << 8) | (readByteFromReg(0x010))) * GYRO_SENSITIVITY; 

    for(int i = 0 ; i < 3 ; i++){
        if(gyro[i] > (Gyro_range/2.0)){
            gyro[i] = gyro[i] - Gyro_range;
        }
    }
    Serial.print("Gyro X : ");
    Serial.print(gyro[0]); Serial.print("\t");
    Serial.print("Gyro Y : ");
    Serial.print(gyro[1]); Serial.print("\t");
    Serial.print("Gyro Z : ");
    Serial.println(gyro[2]);
}    

void bmx160::get_mag_data(){
    mag[0] = ((readByteFromReg(0x05) << 8) | (readByteFromReg(0x04)))*0.3F;
    mag[1] = ((readByteFromReg(0x05) << 8) | (readByteFromReg(0x04)))*0.3F;
    mag[2] = ((readByteFromReg(0x05) << 8) | (readByteFromReg(0x04)))*0.3F;
    for(int i = 0 ; i < 2 ; i++){
        if(mag[i] > (Mag_range[i]/2)){
            mag[i] = mag[i] - Mag_range[0];
        }
    }
    if(mag[2] > (Mag_range[1]/2)){
            mag[2] = mag[2] - Mag_range[1];
        }
    Serial.print("Mag X : ");
    Serial.print(mag[0]); Serial.print("uT \t");
    Serial.print("Mag Y : ");
    Serial.print(mag[1]); Serial.print("uT \t");
    Serial.print("Mag Z : ");
    Serial.println(mag[2]);
}

byte bmx160::read_acc_powermode(){
    byte pmu_status = readByteFromReg(0x03);
    return ((pmu_status&ACC_PMU_STATUS_BM)>>4);
}

byte bmx160::read_gyro_powermode(){
    byte pmu_status = readByteFromReg(0x03);
    return ((pmu_status&GYRO_PMU_STATUS_BM)>>2);
}

byte bmx160::read_mag_if_powermode(){
    byte pmu_status = readByteFromReg(0x03);
    return (pmu_status&MAG_IF_PMU_STATUS_BM);
}


#endif