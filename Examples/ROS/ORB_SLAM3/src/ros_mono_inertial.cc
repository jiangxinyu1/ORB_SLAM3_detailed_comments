/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>

#include<opencv2/core/core.hpp>

#include "include/System.h"
#include "../include/ImuTypes.h"
#include "../static_imu_init.h"

using namespace std;

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);
    queue<sensor_msgs::ImuConstPtr> imuBuf;
    queue<sensor_msgs::ImuConstPtr> imuBufForInit;
    std::mutex mBufMutex;
    std::mutex mBufMutexForInit;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM,
                 ImuGrabber *pImuGb,
                 const bool bClahe): mpSLAM(pSLAM), mpImuGb(pImuGb), mbClahe(bClahe){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();
    void InitImu();

    // 存储图像的双端队列
    std::queue<sensor_msgs::ImageConstPtr> img0Buf;
    std::mutex mBufMutex;
   
    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "Mono_Inertial");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  bool bEqual = false;
  if(argc < 3 || argc > 4)
  {
    cerr << endl << "Usage: rosrun ORB_SLAM3 Mono_Inertial path_to_vocabulary path_to_settings [do_equalize]" << endl;
    ros::shutdown();
    return 1;
  }

  if(argc==4)
  {
    std::string sbEqual(argv[3]);
    if(sbEqual == "true")
      bEqual = true;
  }

  // Create SLAM system.
  // It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(argv[1],
                         argv[2],
                         ORB_SLAM3::System::IMU_MONOCULAR,
                         true);


  ImuGrabber imugb;
  ImageGrabber igb(&SLAM,&imugb,bEqual);
  
  // Maximum delay, 5 seconds
  // ros::Subscriber sub_imu = n.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb); 
  // ros::Subscriber sub_img0 = n.subscribe("/camera/image_raw", 100, &ImageGrabber::GrabImage,&igb);
  // 以下为EUROC
  ros::Subscriber sub_imu = n.subscribe("/stereo_inertial_publisher/imu",
                                        1000,
                                        &ImuGrabber::GrabImu,
                                        &imugb);
  ros::Subscriber sub_img0 = n.subscribe("/stereo_inertial_publisher/left/image_rect",
                                         100,
                                         &ImageGrabber::GrabImage,
                                         &igb);
  std::thread sync_thread(&ImageGrabber::SyncWithImu,&igb);

  std::thread imu_init_thread(&ImageGrabber::InitImu,&igb);


  ros::spin();
  return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  // 如果图像队列非空，会直接丢弃最老的图像，然后把最新的图像塞进去
  mBufMutex.lock();
  if (!img0Buf.empty())
  {
    img0Buf.pop();
  }

  img0Buf.push(img_msg);
  mBufMutex.unlock();
}


/**
 * 获取图像数据 ros -> cv mat
 * @param img_msg
 * @return
 */
cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(img_msg,
                                  sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  
  if(cv_ptr->image.type()==0)
  {
    return cv_ptr->image.clone();
  }
  else
  {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}


/**
 * 搞出图像和imu原始数据，开始跟踪
 * step 1 ： 判断时间戳
 * step 2 ： 获取队列里最老的图像，pop一个
 * step 3 ： 获取imu队列中小于图像时间戳之前的所有imu原始数据
 * step 4 ： 开始跟踪
 */
void ImageGrabber::SyncWithImu()
{
  while(1)
  {
    cv::Mat im;
    double tIm = 0;
    if (!img0Buf.empty()&&!mpImuGb->imuBuf.empty())
    {
      // step 1 ：判断时间戳
      // 获取图像队列最开始的时间戳
      tIm = img0Buf.front()->header.stamp.toSec();
      if( tIm > mpImuGb->imuBuf.back()->header.stamp.toSec())
      {
        /*
         * 如果图像队列最老的时间戳比IMU队列最新的时间戳要新，跳过
         * image:          /------------/
         * imu:  /------/
         */
        continue;
      }

      // step 2 ：获取队列里最老的图像，pop一个
      {
        this->mBufMutex.lock();
        im = GetImage(img0Buf.front());
        img0Buf.pop();
        this->mBufMutex.unlock();
      }

      // step 3 ： 获取imu队列中小于图像时间戳之前的所有imu原始数据
      std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mpImuGb->mBufMutex.lock();
      if(!mpImuGb->imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        /*
         * image:        /
         * imu:  /=======---------/
         */
        while( !mpImuGb->imuBuf.empty() &&
               mpImuGb->imuBuf.front()->header.stamp.toSec() <= tIm )
        {
          // todo ： 时间补偿 该值由 kalibr 的标定报告给出
          double t = mpImuGb->imuBuf.front()->header.stamp.toSec() + 0.066;
          cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x,
                          mpImuGb->imuBuf.front()->linear_acceleration.y,
                          mpImuGb->imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x,
                          mpImuGb->imuBuf.front()->angular_velocity.y,
                          mpImuGb->imuBuf.front()->angular_velocity.z);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
          mpImuGb->imuBuf.pop();
        }
      }
      mpImuGb->mBufMutex.unlock();

      // 默认是false
      if(mbClahe)
      {
        mClahe->apply(im,im);
      }

      // step 4 ： 开始跟踪
      mpSLAM->TrackMonocular(im,tIm,vImuMeas);
    }

    std::chrono::milliseconds tSleep(1);
    std::this_thread::sleep_for(tSleep);
  }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  mBufMutex.unlock();

  mBufMutexForInit.lock();
  imuBufForInit.push(imu_msg);
  mBufMutexForInit.unlock();
  return;
}

void ImageGrabber::InitImu()
{
  sad::StaticIMUInit imu_init;
  while(1)
  {
    if ( ! imu_init.InitSuccess() )
    {
      mpImuGb->mBufMutexForInit.lock();
      while ( !mpImuGb->imuBufForInit.empty() )
      {
        // 通过kalibr标定OAK相机，需要做时间补偿
        double t = (mpImuGb->imuBufForInit.front()->header.stamp.toSec()+ 0.066);
        cv::Point3f acc(mpImuGb->imuBufForInit.front()->linear_acceleration.x,
                        mpImuGb->imuBufForInit.front()->linear_acceleration.y,
                        mpImuGb->imuBufForInit.front()->linear_acceleration.z);
        cv::Point3f gyr(mpImuGb->imuBufForInit.front()->angular_velocity.x,
                        mpImuGb->imuBufForInit.front()->angular_velocity.y,
                        mpImuGb->imuBufForInit.front()->angular_velocity.z);
        imu_init.AddIMU(ORB_SLAM3::IMU::Point(acc,gyr,t));
        mpImuGb->imuBufForInit.pop();
      }
      mpImuGb->mBufMutexForInit.unlock();
    }
    std::chrono::milliseconds tSleep(10);
    std::this_thread::sleep_for(tSleep);
  }
}


