using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using HumanoidRobot.Model.iRobot;
using HumanoidRobot.Model.OWI535RoboticArm;
using Microsoft.Kinect;
using System.Windows;
using System.Windows.Media.Imaging;
using System.Globalization;
using System.IO;
using NAudio.Wave;
using Microsoft.Speech.Recognition;
using System.Windows.Documents;
using System.Windows.Threading;
using System.Diagnostics;
using HumanoidRobot.Model;
using System.Windows.Controls;

namespace HumanoidRobot
{
    [CLSCompliantAttribute(false)]
    public static class Functions
    {
        #region Declarations

        public static bool FollowOn = false;
        public static int iRobotCurrentSpeed = 0;

        // Create Thread arrays for the robot.
        public static Thread[] LeftArmThreadArray = new Thread[1];
        public static Thread[] RightArmThreadArray = new Thread[1];
        public static Thread[] iRobotThreadArray = new Thread[1];
        public static Thread[] KinectThreadArray = new Thread[1];
        public static Thread[] KinectImageThreadArray = new Thread[1];
        public static Thread[] DanceThreadArray = new Thread[1];
        public static Thread[] FollowThreadArray = new Thread[1];

        public static Stopwatch followTimer = new Stopwatch();
        public static Stopwatch danceTimer = new Stopwatch();

        public static MediaElement musicElement = new MediaElement { Width = 0, Height = 0 };

        /// <summary>
        /// Arm Controller Costructors.
        /// </summary>
        public static ArmController leftArm = new ArmController(4711, 0, 0);
        public static ArmController rightArm = new ArmController(4711, 0, 1);

        /// <summary>
        /// iRobot Create Constructor.
        /// </summary>
        public static iRobot irobot = new iRobot();

        #region Kinect Declarations

        /// <summary>
        /// Thread that is reading audio from Kinect stream.
        /// </summary>
        public static Thread readingThread;

        /// <summary>
        /// <code>true</code> if audio is currently being read from Kinect stream, <code>false</code> otherwise.
        /// </summary>
        public static bool reading;

        /// <summary>
        /// Kinect sensor Constructor.
        /// </summary>
        public static KinectSensor sensor;

        public static Skeleton[] skeletons = new Skeleton[0];

        /// <summary>
        /// Speech recognition engine using audio data from Kinect.
        /// </summary>
        public static SpeechRecognitionEngine speechEngine;

        /// <summary>
        /// Intermediate storage for the color data received from the camera
        /// </summary>
        public static byte[] colorPixels;

        /// <summary>
        /// Intermediate storage for the depth data received from the camera
        /// </summary>
        public static DepthImagePixel[] depthPixels;

        /// <summary>
        /// Bitmap that will hold color information
        /// </summary>
        public static WriteableBitmap colorBitmap;

        /// <summary>
        /// Initializes a new instance of the HumanoidRobot class.
        /// </summary>
        /// 
        public static double accumulatedSquareSum;

        /// <summary>
        /// Number of audio samples accumulated so far to compute the next energy value.
        /// </summary>
        public static int accumulatedSampleCount;

        public static byte[] rawPixelData;
        public static byte[] pixelData;

        #endregion Kinect Declarations

        #endregion Declarations

        #region Left Arm

        /// <summary>
        /// Returns the left arm seconds.
        /// </summary>
        /// <param name="LeftArmSecondsStringValue">The left arm seconds string value.</param>
        /// <returns></returns>
        public static int getLeftArmSeconds(string LeftArmSecondsStringValue)
        {
            int seconds;
            Int32.TryParse(LeftArmSecondsStringValue, out seconds);
            return (seconds * 1000);
        }

        public static void LeftLightsOn()
        {
            LeftArmThreadArray[0] = new Thread(() => leftArm.LightOn());
            LeftArmThreadArray[0].Start();
        }

        public static void LeftLightsOff()
        {
            LeftArmThreadArray[0] = new Thread(() => leftArm.LightOff());
            LeftArmThreadArray[0].Start();
        }

        public static void LeftGrippersOpen(int leftArmSeconds)
        {
            LeftArmThreadArray[0] = new Thread(() => leftArm.ClawOpen(leftArmSeconds));
            LeftArmThreadArray[0].Start();
        }

        public static void LeftGrippersClose(int leftArmSeconds)
        {
            LeftArmThreadArray[0] = new Thread(() => leftArm.ClawClose(leftArmSeconds));
            LeftArmThreadArray[0].Start();
        }

        public static void LeftGrippersUp(int leftArmSeconds)
        {
            LeftArmThreadArray[0] = new Thread(() => leftArm.ClawUp(leftArmSeconds));
            LeftArmThreadArray[0].Start();
        }

        public static void LeftGrippersDown(int leftArmSeconds)
        {
            LeftArmThreadArray[0] = new Thread(() => leftArm.ClawDown(leftArmSeconds));
            LeftArmThreadArray[0].Start();
        }

        public static void LeftElbowUp(int leftArmSeconds)
        {
            LeftArmThreadArray[0] = new Thread(() => leftArm.ElbowUp(leftArmSeconds));
            LeftArmThreadArray[0].Start();
        }

        public static void LeftElbowDown(int leftArmSeconds)
        {
            LeftArmThreadArray[0] = new Thread(() => leftArm.ElbowDown(leftArmSeconds));
            LeftArmThreadArray[0].Start();
        }

        public static void LeftArmUp(int leftArmSeconds)
        {
            LeftArmThreadArray[0] = new Thread(() => leftArm.ArmUp(leftArmSeconds));
            LeftArmThreadArray[0].Start();
        }

        public static void LeftArmDown(int leftArmSeconds)
        {
            LeftArmThreadArray[0] = new Thread(() => leftArm.ArmDown(leftArmSeconds));
            LeftArmThreadArray[0].Start();
        }

        public static void LeftArmRotateLeft(int leftArmSeconds)
        {
            LeftArmThreadArray[0] = new Thread(() => leftArm.RotateLeft(leftArmSeconds));
            LeftArmThreadArray[0].Start();
        }

        public static void LeftArmRotateRight(int leftArmSeconds)
        {
            LeftArmThreadArray[0] = new Thread(() => leftArm.RotateRight(leftArmSeconds));
            LeftArmThreadArray[0].Start();
        }

        public static void LeftHandshake()
        {
            LeftArmThreadArray[0] = new Thread(() => leftArm.Handshake());
            LeftArmThreadArray[0].Start();
        }

        public static void LeftArmStop()
        {
            if (LeftArmThreadArray[0] != null)
            {
                LeftArmThreadArray[0].Abort();
                leftArm.Reset();
            }
        }

        public static void LeftEyeFlash(int times)
        {
            LeftArmThreadArray[0] = new Thread(() => leftArm.FlashEye(times));
            LeftArmThreadArray[0].Start();
        }

        #endregion Left Arm

        #region Right Arm

        /// <summary>
        /// Returns the right arm seconds.
        /// </summary>
        /// <param name="RightArmSecondsStringValue">The right arm seconds string value.</param>
        /// <returns></returns>
        public static int getRightArmSeconds(string RightArmSecondsStringValue)
        {
            int seconds;
            Int32.TryParse(RightArmSecondsStringValue, out seconds);
            return (seconds * 1000);
        }

        public static void RightLightsOn()
        {
            RightArmThreadArray[0] = new Thread(() => rightArm.LightOn());
            RightArmThreadArray[0].Start();
        }

        public static void RightLightsOff()
        {
            RightArmThreadArray[0] = new Thread(() => rightArm.LightOff());
            RightArmThreadArray[0].Start();
        }

        public static void RightGrippersOpen(int rightArmSeconds)
        {
            RightArmThreadArray[0] = new Thread(() => rightArm.ClawOpen(rightArmSeconds));
            RightArmThreadArray[0].Start();
        }

        public static void RightGrippersClose(int rightArmSeconds)
        {
            RightArmThreadArray[0] = new Thread(() => rightArm.ClawClose(rightArmSeconds));
            RightArmThreadArray[0].Start();
        }

        public static void RightGrippersUp(int rightArmSeconds)
        {
            RightArmThreadArray[0] = new Thread(() => rightArm.ClawUp(rightArmSeconds));
            RightArmThreadArray[0].Start();
        }

        public static void RightGrippersDown(int rightArmSeconds)
        {
            RightArmThreadArray[0] = new Thread(() => rightArm.ClawDown(rightArmSeconds));
            RightArmThreadArray[0].Start();
        }

        public static void RightElbowUp(int rightArmSeconds)
        {
            RightArmThreadArray[0] = new Thread(() => rightArm.ElbowUp(rightArmSeconds));
            RightArmThreadArray[0].Start();
        }

        public static void RightElbowDown(int rightArmSeconds)
        {
            RightArmThreadArray[0] = new Thread(() => rightArm.ElbowDown(rightArmSeconds));
            RightArmThreadArray[0].Start();
        }

        public static void RightArmUp(int rightArmSeconds)
        {
            RightArmThreadArray[0] = new Thread(() => rightArm.ArmUp(rightArmSeconds));
            RightArmThreadArray[0].Start();
        }

        public static void RightArmDown(int rightArmSeconds)
        {
            RightArmThreadArray[0] = new Thread(() => rightArm.ArmDown(rightArmSeconds));
            RightArmThreadArray[0].Start();
        }

        public static void RightArmRotateLeft(int rightArmSeconds)
        {
            RightArmThreadArray[0] = new Thread(() => rightArm.RotateLeft(rightArmSeconds));
            RightArmThreadArray[0].Start();
        }

        public static void RightArmRotateRight(int rightArmSeconds)
        {
            RightArmThreadArray[0] = new Thread(() => rightArm.RotateRight(rightArmSeconds));
            RightArmThreadArray[0].Start();
        }

        public static void RightHandshake()
        {
            RightArmThreadArray[0] = new Thread(() => rightArm.Handshake());
            RightArmThreadArray[0].Start();
        }

        public static void RightArmStop()
        {
            if (RightArmThreadArray[0] != null)
            {
                RightArmThreadArray[0].Abort();
                rightArm.Reset();
            }
        }

        public static void RightEyeFlash(int times)
        {
                RightArmThreadArray[0] = new Thread(() => rightArm.FlashEye(times));
                RightArmThreadArray[0].Start();
        }

        #endregion Left Arm

        #region Flash Eyes

        // Flash Eyes function.
        public static void FlashEyes(int times)
        {
            Functions.LeftEyeFlash(times);
            Functions.RightEyeFlash(times);
        }

        #endregion Flash Eyes

        #region iRobot Create

        #region iRobot Connections

        /// <summary>
        /// Try to connect to iRobot Create.
        /// </summary>
        public static string iRobotTryConnect(string COMPort)
        {

            string output="";
            try
            {
                if (irobot.Connect(COMPort))
                {
                    output = "iRobot Connected...";
                }
            }
            catch
            {
                output = "No iRobot found on this port!\r\n";
            }

            return output;
        }

        /// <summary>
        /// Try to disconnect from iRobot Create.
        /// </summary>
        public static string iRobotTryDisconnect()
        {
            string output = "";
            if (!irobot.IsOpen())
                return output;
            try
            {
                irobot.Disconnect();
                output = "Disconnected!!\r\n";
            }
            catch
            {
                output = "Error disconnecting";
            }
            return output;
        }

        #endregion iRobot Connections

        #region iRobot Move Functions

        /// <summary>
        /// iRobot Create move forward at a given Velocity.
        /// </summary>
        public static void iRobotMoveForward(int velocity)
        {
            if(irobot.IsOpen())
            {
                iRobotThreadArray[0] = new Thread(() => irobot.move(velocity,0));
                iRobotThreadArray[0].Start();
            }
        }

        /// <summary>
        /// iRobot Create move backward at a given Velocity.
        /// </summary>
        public static void iRobotMoveBackward(int velocity)
        {
            if (irobot.IsOpen())
            {
                iRobotThreadArray[0] = new Thread(() => irobot.move(-velocity, 0));
                iRobotThreadArray[0].Start();
            }
        }

        /// <summary>
        /// iRobot Create Turn Left.
        /// </summary>
        public static void iRobotTurnLeft(int velocity)
        {
            if (irobot.IsOpen())
            {
                iRobotThreadArray[0] = new Thread(() => irobot.moveLeft(velocity));
                iRobotThreadArray[0].Start();
            }
        }

        /// <summary>
        /// iRobot Create Turn Right.
        /// </summary>
        public static void iRobotTurnRight(int velocity)
        {
            if (irobot.IsOpen())
            {
                iRobotThreadArray[0] = new Thread(() => irobot.moveRight(velocity));
                iRobotThreadArray[0].Start();
            }
        }

        /// <summary>
        /// iRobot Create Stop.
        /// </summary>
        public static void iRobotStop()
        {
            if (irobot.IsOpen())
            {
                iRobotThreadArray[0] = new Thread(() => irobot.moveStop());
                iRobotThreadArray[0].Start();
            }
        }

        #endregion iRobot Move Functions

        #endregion iRobot Create

        #region Kinect

        #region Power

        public static void KinectPowerOn()
        {
            if (irobot.IsOpen())
            {
                KinectThreadArray[0] = new Thread(() => irobot.KinectPowerOn());
                KinectThreadArray[0].Start();
            }
        }

        public static void KinectPowerOff()
        {
            if (irobot.IsOpen())
            {
                KinectThreadArray[0] = new Thread(() => irobot.KinectPowerOff());
                KinectThreadArray[0].Start();
            }
        }

        #endregion Power

        #region Tilt

        public static void KinectTiltUp(int angle)
        {
            KinectThreadArray[0] = new Thread(() => KinectSensorUp(sensor, angle));
            KinectThreadArray[0].Start();
        }

        public static void KinectTiltDown(int angle)
        {
            KinectThreadArray[0] = new Thread(() => KinectSensorDown(sensor, angle));
            KinectThreadArray[0].Start();
        }

        public static void KinectTiltAngle(int angle)
        {
            KinectThreadArray[0] = new Thread(() => KinectSensorTiltAngle(sensor, angle));
            KinectThreadArray[0].Start();
        }

        /// <summary>
        /// Tilts the Kinect Sensor up by x degrees, where x is the angle in int.
        /// </summary>
        /// <param name="angle">Tilt angle</param>
        private static void KinectSensorUp(KinectSensor sensor, int angle)
        {
            if ((sensor.ElevationAngle + angle) <= sensor.MaxElevationAngle)
            {
                sensor.ElevationAngle += angle;
            }
            else
            {
                sensor.ElevationAngle = sensor.MaxElevationAngle;
            }
        }

        /// <summary>
        /// Tilts the Kinect Sensor down by x degrees, where x is the angle in int.
        /// </summary>
        /// <param name="angle">Tilt angle</param>
        private static void KinectSensorDown(KinectSensor sensor, int angle)
        {
            if ((sensor.ElevationAngle - angle) >= sensor.MinElevationAngle)
            {
                sensor.ElevationAngle -= angle;
            }
            else
            {
                sensor.ElevationAngle = sensor.MinElevationAngle;
            }
        }

        /// <summary>
        /// Tilts the Kinect Sensor to a specified angle x degrees, ranging from -27 to 27 degrees.
        /// </summary>
        /// <param name="angle">Tilt angle</param>
        private static void KinectSensorTiltAngle(KinectSensor sensor, int angle)
        {
            sensor.ElevationAngle = angle;
        }

        #endregion Tilt

        #region Vision

        public static void KinectTakeScreenshot()
        {
            if (null == sensor)
            {
                //this.statusBarText.Text = Properties.Resources.ConnectDeviceFirst;
                return;
            }

            // create a png bitmap encoder which knows how to save a .png file
            BitmapEncoder encoder = new PngBitmapEncoder();

            // create frame from the writable bitmap and add to encoder
            encoder.Frames.Add(BitmapFrame.Create(colorBitmap));
            encoder.Frames.Add(BitmapFrame.Create(Functions.colorBitmap));

            string time = System.DateTime.Now.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);

            string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);

            string path = Path.Combine(myPhotos, "KinectSnapshot-" + time + ".png");

            // write the new file to disk
            try
            {
                using (FileStream fs = new FileStream(path, FileMode.Create))
                {
                    encoder.Save(fs);
                }
            }
            catch (IOException)
            {}
        }

        public static void KinectActivateDepthMode()
        {
            sensor.ColorStream.Disable();
            sensor.DepthStream.Enable();
        }

        public static void KinectActivateColorMode()
        {
            sensor.DepthStream.Disable();
            sensor.ColorStream.Enable();
        }

        /// <summary>
        /// Event handler for Kinect sensor's DepthFrameReady event
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        public static void SensorDepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (depthFrame != null)
                {
                    // Copy the pixel data from the image to a temporary array
                    depthFrame.CopyDepthImagePixelDataTo(depthPixels);

                    // Get the min and max reliable depth for the current frame
                    int minDepth = depthFrame.MinDepth;
                    int maxDepth = depthFrame.MaxDepth;

                    // Convert the depth to RGB
                    int colorPixelIndex = 0;
                    for (int i = 0; i < depthPixels.Length; ++i)
                    {
                        // Get the depth for this pixel
                        short depth = depthPixels[i].Depth;

                        // To convert to a byte, we're discarding the most-significant
                        // rather than least-significant bits.
                        // We're preserving detail, although the intensity will "wrap."
                        // Values outside the reliable depth range are mapped to 0 (black).

                        // Note: Using conditionals in this loop could degrade performance.
                        // Consider using a lookup table instead when writing production code.
                        // See the KinectDepthViewer class used by the KinectExplorer sample
                        // for a lookup table example.
                        byte intensity = (byte)(depth >= minDepth && depth <= maxDepth ? depth : 0);

                        // Write out blue byte
                        colorPixels[colorPixelIndex++] = intensity;

                        // Write out green byte
                        colorPixels[colorPixelIndex++] = intensity;

                        // Write out red byte                        
                        colorPixels[colorPixelIndex++] = intensity;

                        // We're outputting BGR, the last byte in the 32 bits is unused so skip it
                        // If we were outputting BGRA, we would write alpha here.
                        ++colorPixelIndex;
                    }

                    // Write the pixel data into our bitmap
                    colorBitmap.WritePixels(
                        new Int32Rect(0, 0, colorBitmap.PixelWidth, colorBitmap.PixelHeight),
                        colorPixels,
                        colorBitmap.PixelWidth * sizeof(int),
                        0);
                }
            }
        }

        /// <summary>
        /// Event handler for Kinect sensor's ColorFrameReady event
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        public static void SensorColorFrameReady(object sender, ColorImageFrameReadyEventArgs e)
        {
            
            using (ColorImageFrame colorFrame = e.OpenColorImageFrame())
            {
                if (colorFrame != null)
                {

                    // Copy the pixel data from the image to a temporary array
                    colorFrame.CopyPixelDataTo(colorPixels);
                    var rect = new Int32Rect(0, 0, colorBitmap.PixelWidth, colorBitmap.PixelHeight);
                    // Write the pixel data into our bitmap
                    //colorBitmap.WritePixels(
                    //    rect,
                    //    colorPixels,
                    //    colorBitmap.PixelWidth * sizeof(int),
                    //    0);
                }
            }
        }

        /// <summary>
        /// Event handler for Kinect sensor's SkeletonFrameReady event
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>

        public static void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    int mySkeleton = -1;

                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);

                    for (int i = 0; i < skeletons.Length; i++)
                    {
                        if ((skeletons[i].Position.X != 0) || (skeletons[i].Position.Y != 0) || (skeletons[i].Position.Z != 0))
                        {
                            if(mySkeleton == -1)
                                mySkeleton = i;
                        }
                    }

                    if (FollowOn)
                    {
                        if (followTimer.ElapsedMilliseconds > 100)
                        {
                            //Console.WriteLine("Timer checkpoint!");
                            if (mySkeleton >= 0)
                            {
                                Console.WriteLine(skeletons[mySkeleton].Position.X + ", " + skeletons[mySkeleton].Position.Y + ", " + skeletons[mySkeleton].Position.Z);
                                followTimer.Reset();
                                followTimer.Start();
                                Console.WriteLine("Timer Reset after follow");
                                FollowThreadArray[0] = new Thread(() => Follow(skeletons[mySkeleton].Position.X, skeletons[mySkeleton].Position.Y, skeletons[mySkeleton].Position.Z));
                                FollowThreadArray[0].Start();
                            }
                            else
                            {
                                followTimer.Reset();
                                followTimer.Start();
                                Console.WriteLine("Timer Reset after stop");
                                iRobotStop();
                            }
                        }
                    }
                }
                else
                {
                    if (FollowOn)
                    {
                        iRobotStop();
                        //Functions.safeSlowdown();
                    }
                }
            }
        }


        #endregion Vision

        // Anton & Marcs work
        #region Follow

        //This method will track and follow a skeleton
        public static void Follow(double x, double y, double z)
        {
            Console.WriteLine("Follow has been called for point: (" + x + ", " + y + ", " + z + ")");
            //These variables can be changed to allow different follow behavior
            double xTolerance = .2;  //The tolerance window for x
            double zTolerance = .1;  //The tolerance window for z
            double followDistance = 1;  //The distance the robot will try to maintain

            int forwardVelocity = 0;  //The current forward velocity of the robot.  This starts at 0 and will update as the method runs.
            int maxForwardVelocity = 100; //The top speed of the robot

            int rotationalVelocity = 30;  //How fast the robot will rotate

            int goFwdAcceleleration = 3;  //This is the forward accelleration (per 1/10 seconds)
            int stopFwdAcceleration = 5;  //This is the stopping decelleration (per 1/10 seconds)
            
            //while (FollowOn)
            //{

            //Step 1 - Check if there is a valid skeleton
            if (x != 0 || y != 0 || z != 0)
            {
                Console.WriteLine("X = " + x + "; Y = " + y + "; Z = " + z);
                //Step 2 - Rotate to center the skeletal point

                //If skeleton too far to the left, turn left until centered
                if (x < (xTolerance * -1))
                {
                    Functions.safeSlowdown();
                    Console.WriteLine("Trying to turn left!");
                    iRobotTurnRight(rotationalVelocity);

                }
                //If skeleton too far to the right, turn right until centered
                else if (x > xTolerance)
                {
                    Functions.safeSlowdown();
                    Console.WriteLine("Trying to turn right!");
                    iRobotTurnLeft(rotationalVelocity);
                }

                //Step 3 - Move to follow

                //If the target is closer to the robot than the follow distance, slow the robot down.
                else if (z <= (followDistance - zTolerance))
                {
                    //Bring Forward Velocity to 0 by steps of 2;
                    //if (forwardVelocity >= stopFwdAcceleration)
                        //forwardVelocity = forwardVelocity - stopFwdAcceleration;
                    //else
                        //forwardVelocity = 0;
                    Console.WriteLine("Moving backwards!");
                    forwardVelocity = maxForwardVelocity;
                    iRobotCurrentSpeed = forwardVelocity;
                    iRobotMoveBackward(forwardVelocity);
                    //Functions.safeSlowdown();
                }
                //If the target is farther away than the follow distance
                else if (z > (followDistance + zTolerance))
                {
                    //Forward Velocity add acceleration value if velocity not maxed out.
                    //if (forwardVelocity < maxForwardVelocity)
                    //forwardVelocity = forwardVelocity + goFwdAcceleleration;
                    Console.WriteLine("Moving forwards!");
                    forwardVelocity = maxForwardVelocity;
                    iRobotCurrentSpeed = forwardVelocity;
                    iRobotMoveForward(forwardVelocity);
                    //iRobotIncrementalMoveForward(forwardVelocity);
                }
                else
                {
                    iRobotStop();
                }

                //Print out the forward velocity
                Console.WriteLine(" Forward Velocity: " + forwardVelocity.ToString());
            }
            //}
        }

        //#region commented code
        //public static void Followold()
        //{
        //    //This method will track and follow a skeleton
        //    double xTolerance = .1;
        //    double zTolerance = .1;
        //    double followDistance = 1;

        //    int forwardVelocity = 0;
        //    int maxForwardVelocity = 30;

        //    int RotationalVelocity = 10;

        //    int goFwdAcceleleration = 3;
        //    int stopFwdAcceleration = 5;
        //    int point = 0;
        //    //Follow for
        //    Stopwatch timer = new Stopwatch();
        //    timer.Start();

        //    ////while (timer.ElapsedMilliseconds < 3000)
        //    ////{
        //    ////    for (int i = 0; i < skeletons.Length; i++)
        //    ////        if(skeletons[i].Position.X != 0 || skeletons[i].Position.Y != 0 || skeletons[i].Position.Z != 0)
        //    ////        {    
        //    ////            Console.WriteLine("X = " + Functions.skeletons[i].Position.X.ToString() + "; Y = " + Functions.skeletons[i].Position.Y.ToString() + "; Z = " + Functions.skeletons[i].Position.Z.ToString());
        //    ////            point = i;
        //    ////        }
        //    ////    //while (Functions.skeletons[0].Position.X == 0 && Functions.skeletons[0].Position.Y == 0 && Functions.skeletons[0].Position.Z == 0)
        //    ////    //{
        //    ////    //    Console.WriteLine("No point received");
        //    ////    //    Console.WriteLine("X = " + Functions.skeletons[0].Position.X.ToString() + "; Y = " + Functions.skeletons[0].Position.Y.ToString() + "; Z = " + Functions.skeletons[0].Position.Z.ToString());
        //    ////    //}
        //    ////    // updates sceleton points every 1/10 second
        //    ////   Console.WriteLine("X = " + Functions.skeletons[point].Position.X.ToString() + "; Y = " + Functions.skeletons[point].Position.Y.ToString() + "; Z = " + Functions.skeletons[point].Position.Z.ToString());

        //    ////    //Step 2 - Rotate to center the skeletal point

        //    ////    //if (Math.Abs(Functions.skeletons[0].Position.X) <= xTolerance)
        //    ////    //{
        //    ////    //    //Bring Rotational Velocity to 0 by steps of 2;
        //    ////    //    if (rotationalVelocity >= stopRotAcceleration)
        //    ////    //        rotationalVelocity = rotationalVelocity - stopRotAcceleration;
        //    ////    //    else if (rotationalVelocity <= -2)
        //    ////    //        rotationalVelocity = rotationalVelocity + stopRotAcceleration;
        //    ////    //    else
        //    ////    //        rotationalVelocity = 0;
        //    ////    //}
        //    ////    //else if (Functions.skeletons[0].Position.X < xTolerance * -1)
        //    ////    //{
        //    ////    //    //Rotational Velocity add -1 if velocity not maxed out.
        //    ////    //    if (rotationalVelocity > maxRotVel * -1)
        //    ////    //        rotationalVelocity = rotationalVelocity - goRotAcceleleration;
        //    ////    //}
        //    ////    //else if (Functions.skeletons[0].Position.X > xTolerance)
        //    ////    //{
        //    ////    //    //Rotational Velocity add 1 if velocity not maxed out.
        //    ////    //    if (rotationalVelocity < maxRotVel)
        //    ////    //        rotationalVelocity = rotationalVelocity + goRotAcceleleration;
        //    ////    //}

        //    ////    //Step 3 - Move to follow 
        //    ////    //If the target is closer to the robot than the follow distance, slow the robot down.
        //    ////   if (Math.Abs(Functions.skeletons[point].Position.Z) <= followDistance - zTolerance)
        //    ////   {
        //    ////       //Bring Forward Velocity to 0 by steps of 2;
        //    ////       if (forwardVelocity >= stopFwdAcceleration)
        //    ////           forwardVelocity = forwardVelocity - stopFwdAcceleration;
        //    ////       else
        //    ////           forwardVelocity = 0;
        //    ////   }
        //    ////   //If the target is farther away than the follow distance
        //    ////   else if (Functions.skeletons[point].Position.Z > followDistance + zTolerance)
        //    ////   {
        //    ////       //Forward Velocity add 1 if velocity not maxed out.
        //    ////       if (forwardVelocity < maxForwardVelocity)
        //    ////           forwardVelocity = forwardVelocity + goFwdAcceleleration;
        //    ////   }
        //    ////   //Send the command to the robot
        //    ////   iRobotMoveForward(forwardVelocity);
        //    ////  //  System.Threading.Thread.Sleep(100);
        //    ////    ////return current forward and rotational velocity to the create

        //    ////    Console.WriteLine(" Forward Velocity: " + forwardVelocity.ToString());
        //    ////}
        //    while (timer.ElapsedMilliseconds < 3000)
        //    {
        //        //for (int i = 0; i < skeletons.Length; i++)
        //        if (skeletons[0].Position.X != 0 || skeletons[0].Position.Y != 0 || skeletons[0].Position.Z != 0)
        //        {
        //            //      Console.WriteLine("X = " + Functions.skeletons[0].Position.X.ToString() + "; Y = " + Functions.skeletons[0].Position.Y.ToString() + "; Z = " + Functions.skeletons[0].Position.Z.ToString());
        //            //point = i;
        //        }
        //        //while (Functions.skeletons[0].Position.X == 0 && Functions.skeletons[0].Position.Y == 0 && Functions.skeletons[0].Position.Z == 0)
        //        //{
        //        //    Console.WriteLine("No point received");
        //        //    Console.WriteLine("X = " + Functions.skeletons[0].Position.X.ToString() + "; Y = " + Functions.skeletons[0].Position.Y.ToString() + "; Z = " + Functions.skeletons[0].Position.Z.ToString());
        //        //}
        //        // updates sceleton points every 1/10 second
        //        //Console.WriteLine("X = " + Functions.skeletons[0].Position.X.ToString() + "; Y = " + Functions.skeletons[0].Position.Y.ToString() + "; Z = " + Functions.skeletons[0].Position.Z.ToString());

        //        //Step 2 - Rotate to center the skeletal point

        //        //if (Math.Abs(Functions.skeletons[0].Position.X) <= xTolerance)
        //        //{
        //        //    //Bring Rotational Velocity to 0 by steps of 2;
        //        //    if (rotationalVelocity >= stopRotAcceleration)
        //        //        rotationalVelocity = rotationalVelocity - stopRotAcceleration;
        //        //    else if (rotationalVelocity <= -2)
        //        //        rotationalVelocity = rotationalVelocity + stopRotAcceleration;
        //        //    else
        //        //        rotationalVelocity = 0;
        //        //}
        //        //else if (Functions.skeletons[0].Position.X < xTolerance * -1)
        //        //{
        //        //    //Rotational Velocity add -1 if velocity not maxed out.
        //        //    if (rotationalVelocity > maxRotVel * -1)
        //        //        rotationalVelocity = rotationalVelocity - goRotAcceleleration;
        //        //}
        //        //else if (Functions.skeletons[0].Position.X > xTolerance)
        //        //{
        //        //    //Rotational Velocity add 1 if velocity not maxed out.
        //        //    if (rotationalVelocity < maxRotVel)
        //        //        rotationalVelocity = rotationalVelocity + goRotAcceleleration;
        //        //}

        //        //Step 3 - Move to follow 
        //        //If the target is closer to the robot than the follow distance, slow the robot down.
        //        if (Math.Abs(Functions.skeletons[0].Position.Z) <= followDistance - zTolerance)
        //        {
        //            //Bring Forward Velocity to 0 by steps of 2;
        //            if (forwardVelocity >= stopFwdAcceleration)
        //                forwardVelocity = forwardVelocity - stopFwdAcceleration;
        //            else
        //                forwardVelocity = 0;
        //        }
        //        //If the target is farther away than the follow distance
        //        else if (Functions.skeletons[0].Position.Z > followDistance + zTolerance)
        //        {
        //            //Forward Velocity add 1 if velocity not maxed out.
        //            if (forwardVelocity < maxForwardVelocity)
        //                forwardVelocity = forwardVelocity + goFwdAcceleleration;
        //        }
        //        //Send the command to the robot
        //        //iRobotMoveForward(forwardVelocity);
        //        //System.Threading.Thread.Sleep(100);
        //        ////return current forward and rotational velocity to the create

        //        Console.WriteLine(" Forward Velocity: " + forwardVelocity.ToString());
        //    }
        //}

        //#endregion commented code

        #endregion Follow

        #region safeSlowdown

        //Slow down the robot safely, without a sudden stop
        public static void safeSlowdown()
        {
            while (iRobotCurrentSpeed > 0)
            {
                if (iRobotCurrentSpeed > 100)
                {
                    iRobotCurrentSpeed = iRobotCurrentSpeed - 20;
                    iRobotMoveForward(iRobotCurrentSpeed);
                }
                else
                {
                    iRobotCurrentSpeed = 0;
                    iRobotStop();
                }
                
                Thread.Sleep(100);
            }
        }

        #endregion safeSlowdown

        #region IncrementalMoveForward

        //public static void iRobotMoveForward(int velocity)
        //{
        //    if (irobot.IsOpen())
        //    {
        //        iRobotThreadArray[0] = new Thread(() => irobot.move(velocity, 0));
        //        iRobotThreadArray[0].Start();
        //    }
        //}

        // Speed up the robot safely, reaching the given velocity gradually.
        public static void iRobotIncrementalMoveForward(int velocity)
        {
            if (irobot.IsOpen())
            {
                while (iRobotCurrentSpeed < velocity)
                {
                    iRobotCurrentSpeed = iRobotCurrentSpeed + 20;
                    iRobotMoveForward(iRobotCurrentSpeed);

                    Thread.Sleep(100);
                }
            }
        }

        #endregion IncrementalMoveForward

        #region IncrementalMoveBackward

        // Speed up the robot safely, reaching the given velocity gradually.
        public static void iRobotIncrementalMoveBackward(int velocity)
        {
            while (iRobotCurrentSpeed < velocity)
            {
                iRobotCurrentSpeed = iRobotCurrentSpeed + 20;
                iRobotMoveBackward(iRobotCurrentSpeed);

                Thread.Sleep(100);
            }
        }

        #endregion IncrementalMoveForward

        #endregion Kinect

        // Jasons work
        #region Dance

        public static void Dance()
        {
            danceTimer.Start();
            if (danceTimer.IsRunning)
                danceTimer.Reset();
            else
                //danceTimer.Start();

            //FlashEyes(1000);
            //WaitForIMilliSeconds(1000);
            RightLightsOn();
            LeftLightsOn();

            //up down set
           
            for (int x = 0; x < 2; x++)
            {
                //arms up
                RightArmUp(4000);
                //LeftArmUp(2000);
                LeftArmDown(4000);
                WaitForIMilliSeconds(5000);

                //arms down
                RightArmDown(3000);
                //LeftArmDown(1500);
                LeftArmUp(3000);
                WaitForIMilliSeconds(3000);
                //Functions.RightArmStop();
                //Functions.LeftArmStop();
            }

            // main dance // so is this
            for (int x = 0; x < 5; x++)
            {
                iRobotTurnLeft(50);
                WaitForIMilliSeconds(1500);
                iRobotTurnRight(50);
                WaitForIMilliSeconds(1500);
            }

            // Spin one circle
            iRobotTurnRight(70);
            WaitForIMilliSeconds(12000);
            iRobotStop();
        }

        private static void WaitForIMilliSeconds(int i)
        {
            danceTimer.Reset();
            danceTimer.Start();
            while (danceTimer.ElapsedMilliseconds < i) { }
        }

        #endregion Dance

        #region Kill All Switch

        public static void KillAllSwitch()
        {
            LeftArmStop();
            RightArmStop();
            LeftLightsOff();
            RightLightsOff();
            iRobotStop();
        }

        #endregion Kil All Switch
        
    }
}
