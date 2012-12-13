using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Media.Imaging;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using HumanoidRobot.Model;

namespace HumanoidRobot
{
    public class HumanoidService : IHumanoidService
    {
        #region Left Arm

        public void LeftLightsOn()
        {
            Functions.LeftLightsOn();
        }

        public void LeftLightsOff()
        {
            Functions.LeftLightsOff();
        }

        public void LeftGrippersOpen(int LeftArmSeconds)
        {
            Functions.LeftGrippersOpen(LeftArmSeconds*1000);
        }

        public void LeftGrippersClose(int LeftArmSeconds)
        {
            Functions.LeftGrippersClose(LeftArmSeconds * 1000);
        }

        public void LeftGrippersUp(int LeftArmSeconds)
        {
            Functions.LeftGrippersUp(LeftArmSeconds * 1000);
        }

        public void LeftGrippersDown(int LeftArmSeconds)
        {
            Functions.LeftGrippersDown(LeftArmSeconds * 1000);
        }

        public void LeftElbowUp(int LeftArmSeconds)
        {
            Functions.LeftElbowUp(LeftArmSeconds * 1000);
        }

        public void LeftElbowDown(int LeftArmSeconds)
        {
            Functions.LeftElbowDown(LeftArmSeconds * 1000);
        }

        public void LeftArmUp(int LeftArmSeconds)
        {
            Functions.LeftArmUp(LeftArmSeconds * 1000);
        }

        public void LeftArmDown(int LeftArmSeconds)
        {
            Functions.LeftElbowDown(LeftArmSeconds * 1000);
        }

        public void LeftArmRotateLeft(int LeftArmSeconds)
        {
            Functions.LeftArmRotateLeft(LeftArmSeconds * 1000);
        }

        public void LeftArmRotateRight(int LeftArmSeconds)
        {
            Functions.LeftArmRotateRight(LeftArmSeconds * 1000);
        }

        public void LeftHandshake()
        {
            Functions.LeftHandshake();
        }

        public void LeftArmStop()
        {
            Functions.LeftArmStop();
        }
        
        #endregion Left Arm

        #region Right Arm

        public void RightLightsOn()
        {
            Functions.RightLightsOn();
        }

        public void RightLightsOff()
        {
            Functions.RightLightsOff();
        }

        public void RightGrippersOpen(int RightArmSeconds)
        {
            Functions.RightGrippersOpen(RightArmSeconds * 1000);
        }

        public void RightGrippersClose(int RightArmSeconds)
        {
            Functions.RightGrippersClose(RightArmSeconds * 1000);
        }

        public void RightGrippersUp(int RightArmSeconds)
        {
            Functions.RightGrippersUp(RightArmSeconds * 1000);
        }

        public void RightGrippersDown(int RightArmSeconds)
        {
            Functions.RightGrippersDown(RightArmSeconds * 1000);
        }

        public void RightElbowUp(int RightArmSeconds)
        {
            Functions.RightElbowUp(RightArmSeconds * 1000);
        }

        public void RightElbowDown(int RightArmSeconds)
        {
            Functions.RightElbowDown(RightArmSeconds * 1000);
        }

        public void RightArmUp(int RightArmSeconds)
        {
            Functions.RightArmUp(RightArmSeconds * 1000);
        }

        public void RightArmDown(int RightArmSeconds)
        {
            Functions.RightElbowDown(RightArmSeconds * 1000);
        }

        public void RightArmRotateLeft(int RightArmSeconds)
        {
            Functions.RightArmRotateLeft(RightArmSeconds * 1000);
        }

        public void RightArmRotateRight(int RightArmSeconds)
        {
            Functions.RightArmRotateRight(RightArmSeconds * 1000);
        }

        public void RightHandshake()
        {
            Functions.RightHandshake();
        }

        public void RightArmStop()
        {
            Functions.RightArmStop();
        }

        #endregion Right Arm

        #region iRobot create

        public string iRobotConnect(string COMPort)
        {
            if (!Functions.irobot.IsOpen())
            {
                return Functions.iRobotTryConnect(COMPort);
            }
            return "";
        }
        
        public string iRobotDisconnect()
        {
            if (Functions.irobot.IsOpen())
            {
                return Functions.iRobotTryDisconnect();
            }
            return "";
        }
        
        public void iRobotMoveForward(int iRobotVelocity)
        {
            Functions.iRobotMoveForward(iRobotVelocity);
        }
        
        public void iRobotMoveBackward(int iRobotVelocity)
        {
            Functions.iRobotMoveBackward(iRobotVelocity);
        }
        
        public void iRobotTurnLeft(int iRobotVelocity)
        {
            Functions.iRobotTurnLeft(iRobotVelocity);
        }
        
        public void iRobotTurnRight(int iRobotVelocity)
        {
            Functions.iRobotTurnRight(iRobotVelocity);
        }
        
        public void iRobotStop()
        {
            Functions.iRobotStop();
        }

        #endregion iRobot create

        #region Kinect

        public void KinectPowerOn()
        {
            Functions.KinectPowerOn();
        }
        
        public void KinectPowerOff()
        {
            Functions.KinectPowerOff();
        }

        public void KinectTiltUp(int angle)
        {
            Functions.KinectTiltUp(angle);
        }

        public void KinectTiltDown(int angle)
        {
            Functions.KinectTiltDown(angle);
        }

        public void KinectTiltAngle(int angle)
        {
            Functions.KinectTiltAngle(angle);
        }

        public void KinectActivateDepthMode()
        {
            Functions.KinectActivateDepthMode();
        }

        public void KinectActivateColorMode()
        {
            Functions.KinectActivateColorMode();
        }

        public void KinectTakeScreenshot()
        {
            Functions.KinectTakeScreenshot();
        }

        public PixelData KinectStuff()
        {
            byte[] reducedColorPixels = QualityReducer.Reduce(Functions.colorPixels);
            //byte[] reducedColorPixels2 = QualityReducer.Reduce2(reducedColorPixels);
            byte[] compressedColorPixels = Compressor.Compress(reducedColorPixels);
            return new PixelData { pixelHeight = Functions.colorBitmap.PixelHeight / 2, pixelWidth = Functions.colorBitmap.PixelWidth / 2, colorPixels = compressedColorPixels };
        }

        public class PixelData
        {
            public int pixelHeight { get; set; }
            public int pixelWidth { get; set; }
            public byte[] colorPixels { get; set; }
        }

        #endregion Kinect

        #region Miscellaneous

        public void FlashEyes(int times)
        {
            Functions.FlashEyes(times);
        }

        public void Dance()
        {
            Functions.Dance();
        }

        public void Follow(bool FollowOn)
        {
            Functions.FollowOn = FollowOn;
        }

        public void KillAllSwitch()
        {
            Functions.KillAllSwitch();
        }

        #endregion Miscellaneous
    }
}
