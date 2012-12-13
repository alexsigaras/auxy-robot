using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.Serialization;
using System.ServiceModel;
using System.Text;
using System.Windows.Media.Imaging;

namespace HumanoidRobot
{
    [ServiceContract]
    public interface IHumanoidService
    {
        #region Left Arm

        [OperationContract]
        void LeftLightsOn();

        [OperationContract]
        void LeftLightsOff();

        [OperationContract]
        void LeftGrippersOpen(int leftArmSeconds);

        [OperationContract]
        void LeftGrippersClose(int leftArmSeconds);

        [OperationContract]
        void LeftGrippersUp(int leftArmSeconds);

        [OperationContract]
        void LeftGrippersDown(int leftArmSeconds);

        [OperationContract]
        void LeftElbowUp(int leftArmSeconds);

        [OperationContract]
        void LeftElbowDown(int leftArmSeconds);
        
        [OperationContract]
        void LeftArmUp(int leftArmSeconds);

        [OperationContract]
        void LeftArmDown(int leftArmSeconds);

        [OperationContract]
        void LeftArmRotateLeft(int leftArmSeconds);

        [OperationContract]
        void LeftArmRotateRight(int leftArmSeconds);

        [OperationContract]
        void LeftHandshake();

        [OperationContract]
        void LeftArmStop();

        #endregion Left Arm

        #region Right Arm

        [OperationContract]
        void RightLightsOn();

        [OperationContract]
        void RightLightsOff();

        [OperationContract]
        void RightGrippersOpen(int seconds);

        [OperationContract]
        void RightGrippersClose(int seconds);

        [OperationContract]
        void RightGrippersUp(int seconds);

        [OperationContract]
        void RightGrippersDown(int seconds);

        [OperationContract]
        void RightElbowUp(int seconds);

        [OperationContract]
        void RightElbowDown(int seconds);

        [OperationContract]
        void RightArmUp(int seconds);

        [OperationContract]
        void RightArmDown(int seconds);

        [OperationContract]
        void RightArmRotateLeft(int seconds);

        [OperationContract]
        void RightArmRotateRight(int seconds);

        [OperationContract]
        void RightHandshake();

        [OperationContract]
        void RightArmStop();

        #endregion Right Arm

        #region iRobot create

        [OperationContract]
        string iRobotConnect(string COMPort);
        
        [OperationContract]
        string iRobotDisconnect();

        [OperationContract]
        void iRobotMoveForward(int iRobotVelocity);

        [OperationContract]
        void iRobotMoveBackward(int iRobotVelocity);
        
        [OperationContract]
        void iRobotTurnLeft(int iRobotVelocity);

        [OperationContract]
        void iRobotTurnRight(int iRobotVelocity);

        [OperationContract]
        void iRobotStop();

        #endregion iRobot create

        #region Kinect

        [OperationContract]
        void KinectPowerOn();

        [OperationContract]
        void KinectPowerOff();

        [OperationContract]
        void KinectTiltUp(int angle);

        [OperationContract]
        void KinectTiltDown(int angle);

        [OperationContract]
        void KinectTiltAngle(int angle);

        [OperationContract]
        void KinectActivateDepthMode();

        [OperationContract]
        void KinectActivateColorMode();

        [OperationContract]
        void KinectTakeScreenshot();

        [OperationContract]
        HumanoidService.PixelData KinectStuff();

        #endregion Kinect

        #region Miscellaneous

        [OperationContract]
        void FlashEyes(int times);

        [OperationContract]
        void Dance();

        [OperationContract]
        void Follow(bool FollowOn);

        [OperationContract]
        void KillAllSwitch();

        #endregion Miscellaneous
    }
}
