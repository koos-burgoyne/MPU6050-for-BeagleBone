#ifndef _IMU_HANDLER_H_
#define _IMU_HANDLER_H_

class ImuHandler {
    public:
        ImuHandler();

        void calibrateImu();
        void read();
        
        // void getAttitude();
    
    private:
        int NUM_IMU_CALIBRATION_VALS = 2000;
};

#endif 