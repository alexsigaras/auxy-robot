
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Collections.ObjectModel;
using System.Threading;
using LibUsbDotNet;
using LibUsbDotNet.Main;
using LibUsbDotNet.Info;
using LibUsbDotNet.LudnMonoLibUsb;

namespace HumanoidRobot.Model.OWI535RoboticArm
{
    /// <summary>
    /// Arm controller class for OWI 535 robotic arm.
    /// </summary>
    public class ArmController
    {
        public UsbDevice MyUsbDevice;
        public bool isLightOn = false;

        /// <summary>
        /// Arm Controller constructor.
        /// </summary>
        /// <param name="VendorID">The vendor id of the OWI 535 Robotic Arm (usually 4711)</param>
        /// <param name="ProductID">The vendor id of the OWI 535 Robotic Arm (usually 0)</param>
        /// <param name="ArmID">The arm id of the OWI 535 Robotic Arm that you want to connect to(0->1st, 1->2nd etc.)</param>
        public ArmController(Int32 VendorID, Int32 ProductID, Int32 ArmID)
        {
            MyUsbDevice = LibUsbDotNet.LibUsb.LibUsbRegistry.DeviceList[ArmID].Device;
	    }

        #region Connections
              
        /// <summary>
        /// Make the arm stop.
        /// </summary>
        public void Reset()
        {           
            sendControl(0x00, 0x00);
        }

        /// <summary>
        /// Disconnect the arm.
        /// </summary>
        private void closeDevice()
        {
            UsbDevice.Exit();
        }

        /// <summary>
        /// Reset and disconnect the arm.
        /// </summary>
        public void Disconnect()
        {
            Reset();
            closeDevice();
        }

        /// <summary>
        /// Send a command to perform.
        /// </summary>
        /// <param name="opcode1"></param>
        /// <param name="opcode2"></param>
        private void sendControl(int opcode1, int opcode2)
        // send a USB control transfer
        {
            int opcode3 = getLightValue(isLightOn);
            byte[] bytes = { Convert.ToByte(opcode1), Convert.ToByte(opcode2), Convert.ToByte(opcode3) };

            try
            {
                int bytesWritten;

                UsbSetupPacket usPacket = new UsbSetupPacket();
                usPacket.RequestType = 0x40;
                usPacket.Request = 0x6;
                usPacket.Value = 0x100;
                usPacket.Index = 0x0;
                usPacket.Length = 0x3;

                MyUsbDevice.ControlTransfer(ref usPacket, bytes, bytes.Length, out bytesWritten);
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
            }
        }

        #endregion Connections

        #region Controls

        #region Claw 1-DOF

        /// <summary>
        /// Close the claw.
        /// </summary>
        /// <param name="sleepTime">The duration of the action executed (ms)</param>
        public void ClawClose(int sleepTime)
        {
            sendControl(0x01, 0x00);
            Thread.Sleep(sleepTime);
            Reset();
        }

        /// <summary>
        /// Open the claw.
        /// </summary>
        /// <param name="sleepTime">The duration of the action executed (ms)</param>
        public void ClawOpen(int sleepTime)
        {
            sendControl(0x02, 0x00);
            Thread.Sleep(sleepTime);
            Reset();
        }

        /// <summary>
        /// Move the claw up.
        /// </summary>
        /// <param name="sleepTime">The duration of the action executed (ms)</param>
        public void ClawUp(int sleepTime)
        {
            sendControl(0x04, 0x00);
            Thread.Sleep(sleepTime);
            Reset();
        }

        /// <summary>
        /// Move the claw down.
        /// </summary>
        /// <param name="sleepTime">The duration of the action executed (ms)</param>
        public void ClawDown(int sleepTime)
        {
            sendControl(0x08, 0x00);
            Thread.Sleep(sleepTime);
            Reset();
        }

        #endregion Claw 1-DOF

        #region Elbow 1-DOF

        /// <summary>
        /// Move the elbow up.
        /// </summary>
        /// <param name="sleepTime">The duration of the action executed (ms)</param>
        public void ElbowUp(int sleepTime)
        {
            sendControl(0x10, 0x00);
            Thread.Sleep(sleepTime);
            Reset();
        }

        /// <summary>
        /// Move the elbow down.
        /// </summary>
        /// <param name="sleepTime">The duration of the action executed (ms)</param>
        public void ElbowDown(int sleepTime)
        {
            sendControl(0x20, 0x00);
            Thread.Sleep(sleepTime);
            Reset();
        }
        #endregion Elbow 1-DOF

        #region Arm 2-DOF

        /// <summary>
        /// Moves the arm up.
        /// </summary>
        /// <param name="sleepTime">The duration of the action executed (ms)</param>
        public void ArmUp(int sleepTime)
        {
            sendControl(0x40, 0x00);
            Thread.Sleep(sleepTime);
            Reset();
        }

        /// <summary>
        /// Moves the arm down.
        /// </summary>
        /// <param name="sleepTime">The duration of the action executed (ms)</param>
        public void ArmDown(int sleepTime)
        {
            sendControl(0x80, 0x00);
            Thread.Sleep(sleepTime);
            Reset();
        }

        /// <summary>
        /// Rotate the arm left.
        /// </summary>
        /// <param name="sleepTime">The duration of the action executed (ms)</param>
        public void RotateLeft(int sleepTime)
        {
            sendControl(0x00, 0x02);
            Thread.Sleep(sleepTime);
            Reset();
        }

        /// <summary>
        /// Rotate the arm right.
        /// </summary>
        /// <param name="sleepTime">The duration of the action executed (ms)</param>
        public void RotateRight(int sleepTime)
        {
            sendControl(0x00, 0x01);
            Thread.Sleep(sleepTime);
            Reset();
        }

        #endregion Arm 2-DOF

        #region Light

        /// <summary>
        /// Turn the light on.
        /// </summary>
        public void LightOn()
        {
            //Console.WriteLine("Turn Light On.");
            isLightOn = true;
            Reset();
        }

        /// <summary>
        /// Turn the light off.
        /// </summary>
        public void LightOff()
        {
            //Console.WriteLine("Turn Light Off.");
            isLightOn = false;
            Reset();
        }

        /// <summary>
        /// Gets the light value.
        /// </summary>
        /// <param name="lightOn"></param>
        /// <returns></returns>
        public int getLightValue(bool lightOn)
        {
            if (lightOn)
                return 0x01;
            else
                return 0x00;
        }
        
        #endregion Light

        #region Handshake

        public void Handshake()
        {
            // Extend hand to grasp.
            ArmDown(2000);
            ClawClose(1000);

            // Shake.
            for (int x = 0; x < 5; x++)
            {
                ElbowDown(600);
                ElbowUp(600);
            }

            // Return to original position. 
            ClawOpen(1000);
            ArmUp(2350);
        }

        public void FlashEye(int times)
        {
            for (int i = 0; i < times; i++)
            {
                LightOn();
                LightOff();    
            }
        }

        #endregion Handshake

        #endregion Controls
    }
}
