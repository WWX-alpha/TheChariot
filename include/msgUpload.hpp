// Code : UTF - 8
#ifndef MSG_UPLOAD_HPP
#define MSG_UPLOAD_HPP

#include "roposensor/Header.hpp"
#include "pros/serial.hpp"

struct upData{
    float odomXpos;
    float odomYpos;
};

namespace RopoSensor{
    class msgUploader
    {
    public:
        msgUploader(std::int8_t upLoaderPort, std::int32_t baudrate)
         : messageSender(upLoaderPort, baudrate),
        data({.odomXpos = 0.0, .odomYpos = 0.0})
        {
            BackgroundTask = new pros::Task(BackgroundTaskFunction,this);
        }

        void Update()
        {
            // msg update
            
            // data processing
            RopoSensor::SafeObject<upData> safeMessage(data);
            RopoSensor::UnionBuffer<RopoSensor::SafeObject<upData>> rawMessage(data);
            // sending message
            auto result = rawMessage.ToBuffer();
            tempBuffer[0] = 0xFC, tempBuffer[1] = 0xFD;
            for (int i = 0; i < result.size(); i++) {
                tempBuffer[2 + i] = result[i];
            }
            messageSender.write(tempBuffer, 2 + result.size());
        }

        static void BackgroundTaskFunction(void *Parameter)
		{
            if(Parameter == nullptr)return;
			msgUploader *This = static_cast<msgUploader *>(Parameter);
			while(true)
			{
                This->Update();
                pros::delay(10);
            }
        }

        upData data;
    
    private:
        pros::Serial messageSender;
        uint8_t tempBuffer[1000];    
        pros::Task *BackgroundTask;
    };
}

#endif //MSG_UPLOAD_HPP