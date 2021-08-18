#include<sensor_msgs/Imu.h>

class IMUGrabber
{
public:
    IMUGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};
