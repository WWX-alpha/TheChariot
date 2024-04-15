// Code : UTF - 8
#ifndef MSG_DOWNLOAD_HPP
#define MSG_DOWNLOAD_HPP

#include "roposensor/Header.hpp"
#include "pros/serial.hpp"

struct downData{
    float errorX;
    float elevate_order;
    float speed_order;
    bool find_flag;
    bool reset_flag;
    bool shoot_flag;
    float roll;
    float pitch;
    float yaw;
};

namespace RopoSensor{
    class msgDownloader
    {
    public:
        msgDownloader(std::int8_t upLoaderPort, std::int32_t baudrate):
        messageReceiver(upLoaderPort, baudrate)
        {
            BackgroundTask = new pros::Task(BackgroundTaskFunction,this);
        }

        void GetRawMessageBuffer(std::array<std::uint8_t, sizeof(RopoSensor::SafeObject<downData>)> &rawMessageBuffer)
        {
            // init var
            size_t ms_timeout = 10;
            char dataByte;
            int cnt = 0;
            enum {
                WAITING_FOR_HEAD1,
                WAITING_FOR_HEAD2,
                READING,
                FINISH,
            } state = WAITING_FOR_HEAD1;
            while (state != FINISH) {
                while (messageReceiver.peek_byte() == -1) {
                    pros::delay(ms_timeout);
                }
                dataByte = messageReceiver.read_byte();

                switch (state) {
                    case WAITING_FOR_HEAD1:
                        if (dataByte == static_cast<char>(0xFC)) {
                            state = WAITING_FOR_HEAD2;
                        }
                        break;
                    case WAITING_FOR_HEAD2:
                        if (dataByte == static_cast<char>(0xFD)) {
                            state = READING;
                            cnt = 0;
                        } else {
                            state = WAITING_FOR_HEAD1;
                        }
                        break;
                    case READING:
                        if (cnt < rawMessageBuffer.size()) {
                            rawMessageBuffer[cnt++] = dataByte;
                        } else {
                            state = FINISH;
                        }
                        break;
                    default:
                        break;
                }
            }
        }
        downData data;

    private:
        pros::Serial messageReceiver;
        
        pros::Task *BackgroundTask;
        std::array<std::uint8_t, sizeof(RopoSensor::SafeObject<downData>)> rawMessageBuffer;

        static void BackgroundTaskFunction(void *Parameter)
		{
            if(Parameter == nullptr)return;
			msgDownloader *This = static_cast<msgDownloader *>(Parameter);

            while(1)
            {
                This->GetRawMessageBuffer(This->rawMessageBuffer);
                RopoSensor::UnionBuffer<RopoSensor::SafeObject<downData>> rawMessage(This->rawMessageBuffer);
                if (!rawMessage.ToObject().CheckSafe()) {
                    // error when check bit sum, drop this frame
                    continue;
                }
                This->data = rawMessage.ToObject().object;
                // use data
            }
        }
    };
}


#endif