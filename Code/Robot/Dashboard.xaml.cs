//------------------------------------------------------------------------------
// <copyright file="HumanoidRobot.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace HumanoidRobot
{
    #region Libraries

    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.IO;
    using System.Text;
    using System.Windows;
    using System.Windows.Documents;
    using System.Windows.Media;
    using Microsoft.Win32;
    using System.Windows.Media.Imaging;
    using System.Globalization;
    using System.Linq;
    using System.Net;
    using System.Media;
    using System.IO.Ports;

    // Multithreading and timer libraries.
    using System.Threading;
    using System.Windows.Threading;
    using System.Timers;
    using System.Diagnostics;
    
    // Kinect libraries.
    using Microsoft.Kinect;
    using Microsoft.Speech.AudioFormat;
    using Microsoft.Speech.Recognition;
    using Microsoft.Speech.Synthesis;
    using NAudio.Wave;

    // Custom libraries.
    using HumanoidRobot.Model;
    using HumanoidRobot.Model.Converse;
    using HumanoidRobot.Model.iRobot;
    using HumanoidRobot.Model.OWI535RoboticArm;
    using HumanoidRobot.Model.SpeechRecognition;
    using System.ServiceModel;
    using System.ServiceModel.Description;
    using MahApps.Metro.Controls;
    using System.ServiceModel.Channels;
    
    #endregion Libraries

    /// <summary>
    /// Interaction logic for Dashboard.xaml
    /// </summary>
    [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Design", "CA1001:TypesThatOwnDisposableFieldsShouldBeDisposable",
        Justification = "In a full-fledged application, the SpeechRecognitionEngine object should be properly disposed. For the sake of simplicity, we're omitting that code in this sample.")]
    [CLSCompliantAttribute(false)]
    public partial class Dashboard : MetroWindow
    {
        // Create Thread arrays for the robot.
        Thread[] DanceThreadArray = new Thread[1];
        Thread[] FollowThreadArray = new Thread[1];

        // Service Host proxy for web application server.
        private ServiceHost HostProxy;

        public Dashboard()
        {
            InitializeComponent();
            InitializeWebApplicationServer();
        }

        private void InitializeWebApplicationServer()
        {
            try
            {
                string address = "http://Kinect-PC:31337/HumanoidService";

                HostProxy = new ServiceHost(typeof(HumanoidService), new Uri(address));

                BasicHttpBinding binding = new BasicHttpBinding() { MaxBufferPoolSize = 0, MaxReceivedMessageSize = 3100000, MaxBufferSize = 3100000 };
                // Add an endpoint using that binding.
                
                HostProxy.AddServiceEndpoint(typeof(IHumanoidService), binding, "endpoint");
                // Enable metadata publishing.
        
                ServiceMetadataBehavior smb = new ServiceMetadataBehavior();
                smb.HttpGetEnabled = true;
                HostProxy.Description.Behaviors.Add(smb);
                
                // Open the ServiceHost to start listening for messages. Since
                // no endpoints are explicitly configured, the runtime will create
                // one endpoint per base address for each service contract implemented
                // by the service.

                HostProxy.Open();
                //MessageBox.Show("The service is ready at " + address);
            }

            catch (AddressAccessDeniedException)
            {
                MessageBox.Show("You need to reserve the address for this service");
                HostProxy = null;
            }

            catch (AddressAlreadyInUseException)
            {
                MessageBox.Show("Something else is already using this address");
                HostProxy = null;
            }

            catch (Exception ex)
            {
                MessageBox.Show("Something bad happened on startup: " + ex.Message);
                HostProxy = null;
            }
        }
        
        /// <summary>
        /// Execute initialization tasks.
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
                ///////////////////////////////////////////////////////////////////////////////
                // Look through all sensors and start the first connected one.
                // This requires that a Kinect is connected at the time of app startup.
                // To make your app robust against plug/unplug, 
                // it is recommended to use KinectSensorChooser provided in Microsoft.Kinect.Toolkit
                foreach (var potentialSensor in KinectSensor.KinectSensors)
                {
                    if (potentialSensor.Status == KinectStatus.Connected)
                    {
                        Functions.sensor = potentialSensor;
                        break;
                    }
                }

                if (null != Functions.sensor)
                {

                    // Turn on the skeleton stream to receive skeleton frames
                    Functions.sensor.SkeletonStream.Enable();

                    // Add an event handler to be called whenever there is new color frame data
                    Functions.sensor.SkeletonFrameReady += Functions.SensorSkeletonFrameReady;

                    // Turn on the color stream to receive color frames
                    Functions.sensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);

                    //this.sensor.ColorStream.Enable(ColorImageFormat.InfraredResolution640x480Fps30);
                    // Turn on the depth stream to receive depth frames
                    // this.sensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);

                    // Allocate space to put the depth pixels we'll receive
                    Functions.depthPixels = new DepthImagePixel[Functions.sensor.DepthStream.FramePixelDataLength];

                    // Allocate space to put the pixels we'll receive
                    Functions.colorPixels = new byte[Functions.sensor.ColorStream.FramePixelDataLength];
                    //  Functions.colorPixelsServer = new byte[Functions.sensor.ColorStream.FramePixelDataLength];

                    // This is the bitmap we'll display on-screen
                    Functions.colorBitmap = new WriteableBitmap(Functions.sensor.ColorStream.FrameWidth, Functions.sensor.ColorStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);

                    // Add an event handler to be called whenever there is new color frame data
                    Functions.sensor.ColorFrameReady += Functions.SensorColorFrameReady;

                    // Add an event handler to be called whenever there is new depth frame data
                    Functions.sensor.DepthFrameReady += Functions.SensorDepthFrameReady;

                    // Start the sensor!
                    try
                    {
                        Functions.sensor.Start();
                    }
                    catch (IOException)
                    {
                        Functions.sensor = null;
                    }
                }

                if (null == Functions.sensor)
                {
                    //this.statusBarText.Text = Properties.Resources.NoKinectReady;
                    return;
                }

                RecognizerInfo ri = GetKinectRecognizer();

                if (null != ri)
                {

                    Functions.speechEngine = new SpeechRecognitionEngine(ri.Id);

                    /****************************************************************/

                    //Use this code to create grammar programmatically rather than from
                    //a grammar file.

                    var orders = new Choices();

                    #region Basic Level Orders

                    #region Left Arm Orders

                    orders.Add(new SemanticResultValue("left gripers open", "LEFT GRIPPERS OPEN"));
                    orders.Add(new SemanticResultValue("left gripers close", "LEFT GRIPPERS CLOSE"));
                    orders.Add(new SemanticResultValue("left gripers up", "LEFT GRIPPERS UP"));
                    orders.Add(new SemanticResultValue("left gripers down", "LEFT GRIPPERS DOWN"));
                    orders.Add(new SemanticResultValue("left elbow up", "LEFT ELBOW UP"));
                    orders.Add(new SemanticResultValue("left elbow down", "LEFT ELBOW DOWN"));
                    orders.Add(new SemanticResultValue("left arm up", "LEFT ARM UP"));
                    orders.Add(new SemanticResultValue("left arm down", "LEFT ARM DOWN"));
                    orders.Add(new SemanticResultValue("left arm left", "LEFT ARM LEFT"));
                    orders.Add(new SemanticResultValue("left arm right", "LEFT ARM RIGHT"));
                    orders.Add(new SemanticResultValue("left lights on", "LEFT LIGHTS ON"));
                    orders.Add(new SemanticResultValue("left lights off", "LEFT LIGHTS OFF"));
                    orders.Add(new SemanticResultValue("left arm stop", "LEFT ARM STOP"));

                    #endregion Left Arm Orders

                    #region Right Arm Orders

                    orders.Add(new SemanticResultValue("right gripers open", "RIGHT GRIPPERS OPEN"));
                    orders.Add(new SemanticResultValue("right gripers close", "RIGHT GRIPPERS CLOSE"));
                    orders.Add(new SemanticResultValue("right gripers up", "RIGHT GRIPPERS UP"));
                    orders.Add(new SemanticResultValue("right gripers down", "RIGHT GRIPPERS DOWN"));
                    orders.Add(new SemanticResultValue("right elbow up", "RIGHT ELBOW UP"));
                    orders.Add(new SemanticResultValue("right elbow down", "RIGHT ELBOW DOWN"));
                    orders.Add(new SemanticResultValue("right arm up", "RIGHT ARM UP"));
                    orders.Add(new SemanticResultValue("right arm down", "RIGHT ARM DOWN"));
                    orders.Add(new SemanticResultValue("right arm left", "RIGHT ARM LEFT"));
                    orders.Add(new SemanticResultValue("right arm right", "RIGHT ARM RIGHT"));
                    orders.Add(new SemanticResultValue("right lights on", "RIGHT LIGHTS ON"));
                    orders.Add(new SemanticResultValue("right lights off", "RIGHT LIGHTS OFF"));
                    orders.Add(new SemanticResultValue("right arm stop", "RIGHT ARM STOP"));

                    #endregion Right Arm Orders

                    #region iRobot Create Orders

                    orders.Add(new SemanticResultValue("forward", "FORWARD"));
                    orders.Add(new SemanticResultValue("backward", "BACKWARD"));
                    orders.Add(new SemanticResultValue("turn left", "TURN LEFT"));
                    orders.Add(new SemanticResultValue("turn right", "TURN RIGHT"));
                    orders.Add(new SemanticResultValue("stop create", "STOP CREATE"));

                    #endregion iRobot Create Orders

                    #region Flash Eyes Orders

                    orders.Add(new SemanticResultValue("flash eyes", "FLASH EYES"));
                    orders.Add(new SemanticResultValue("stop flash eyes", "STOP FLASH EYES"));

                    #endregion Flash Eyes Orders

                    #region Kill Al Switch Orders

                    orders.Add(new SemanticResultValue("stop", "STOP"));

                    #endregion Kill Al Switch Orders

                    #endregion Basic Level Orders

                    #region Higher Level Orders

                    #region Handshake Orders

                    orders.Add(new SemanticResultValue("right shake my hand", "RIGHT SHAKE MY HAND"));
                    orders.Add(new SemanticResultValue("left shake my hand", "LEFT SHAKE MY HAND"));

                    #endregion Handshake Orders

                    #region Dance Orders

                    orders.Add(new SemanticResultValue("robot dance", "ROBOT DANCE"));
                    orders.Add(new SemanticResultValue("stop dance", "STOP DANCE"));

                    #endregion Dance Orders

                    #region Follow Orders

                    orders.Add(new SemanticResultValue("follow me", "FOLLOW ME"));
                    orders.Add(new SemanticResultValue("stop follow", "STOP FOLLOW"));

                    #endregion Follow Orders

                    #region Converse Orders

                    orders.Add(new SemanticResultValue("robot", "ROBOT"));

                    #endregion Converse Orders

                    #endregion Higher Level Orders

                    var gb = new GrammarBuilder { Culture = ri.Culture };
                    gb.Append(orders);

                    var g = new Grammar(gb);
                    Functions.speechEngine.LoadGrammar(g);

                    Functions.speechEngine.SpeechRecognized += SpeechRecognized;
                    Functions.speechEngine.SpeechRecognitionRejected += SpeechRejected;

                    Functions.speechEngine.SetInputToAudioStream(
                        Functions.sensor.AudioSource.Start(), new SpeechAudioFormatInfo(EncodingFormat.Pcm, 16000, 16, 1, 32000, 2, null));
                    Functions.speechEngine.RecognizeAsync(RecognizeMode.Multiple);
                }
                else
                {
                    //this.statusBarText.Text = Properties.Resources.NoSpeechRecognizer;
                }
            
                // Set the image we display to point to the bitmap where we'll put the image data
                //this.KinectImage.Source = Functions.colorBitmap;
        }

        /// <summary>
        /// Execute uninitialization tasks.
        /// </summary>
        /// <param name="sender">object sending the event.</param>
        /// <param name="e">event arguments.</param>
        private void WindowClosing(object sender, CancelEventArgs e)
        {
            // Tell audio reading thread to stop and wait for it to finish.
            Functions.reading = false;
            if (null != Functions.readingThread)
            {
                Functions.readingThread.Join();
            }

            if (null != Functions.sensor)
            {
                Functions.sensor.AudioSource.Stop();
                Functions.sensor.Stop();
                Functions.sensor = null;
            }

            if (null != Functions.speechEngine)
            {
                Functions.speechEngine.SpeechRecognized -= SpeechRecognized;
                Functions.speechEngine.SpeechRecognitionRejected -= SpeechRejected;
                Functions.speechEngine.RecognizeAsyncStop();
            }

            // Kill all running threads.
            Functions.KillAllSwitch();
        }

        #region Left Arm

        #region Left Arm UI

        #region Left Arm Lights

        /// <summary>
        /// Left arm lights on.
        /// </summary>
        private void LeftEyeLight_Checked(object sender, RoutedEventArgs e)
        {
            Functions.LeftLightsOn();
        }

        /// <summary>
        /// Left arm lights off.
        /// </summary>
        private void LeftEyeLight_Unchecked(object sender, RoutedEventArgs e)
        {
            Functions.LeftLightsOff();
        }

        #endregion Left Arm Lights

        #region Left Arm Claw

        /// <summary>
        /// Left arm claw open.
        /// </summary>
        private void LeftArmClawOpenBtn_Click(object sender, RoutedEventArgs e)
        {
            int leftArmSeconds = getLeftArmSeconds();
            Functions.LeftGrippersOpen(leftArmSeconds);
        }

        /// <summary>
        /// Left arm claw close.
        /// </summary>
        private void LeftArmClawCloseBtn_Click(object sender, RoutedEventArgs e)
        {
            int leftArmSeconds = getLeftArmSeconds();
            Functions.LeftGrippersClose(leftArmSeconds);
        }

        /// <summary>
        /// Left arm claw up.
        /// </summary>
        private void LeftArmClawMoveUpBtn_Click(object sender, RoutedEventArgs e)
        {
            int leftArmSeconds = getLeftArmSeconds();
            Functions.LeftGrippersUp(leftArmSeconds);
        }

        /// <summary>
        /// Left arm claw down.
        /// </summary>
        private void LeftArmClawMoveDownBtn_Click(object sender, RoutedEventArgs e)
        {
            int leftArmSeconds = getLeftArmSeconds();
            Functions.LeftGrippersDown(leftArmSeconds);
        }

        #endregion Left Arm Claw

        #region Left Arm Elbow

        /// <summary>
        /// Move left arm elbow up.
        /// </summary> 
        private void LeftArmElbowMoveUpBtn_Click(object sender, RoutedEventArgs e)
        {
            int leftArmSeconds = getLeftArmSeconds();
            Functions.LeftElbowUp(leftArmSeconds);
        }

        /// <summary>
        /// Move left arm elbow down.
        /// </summary>
        private void LeftArmElbowMoveDownBtn_Click(object sender, RoutedEventArgs e)
        {
            int leftArmSeconds = getLeftArmSeconds();
            Functions.LeftElbowDown(leftArmSeconds);
        }

        #endregion Left Arm Elbow

        #region Left Arm Arm

        /// <summary>
        /// Rotate left arm left.
        /// </summary>
        private void LeftArmArmRotateLeftBtn_Click(object sender, RoutedEventArgs e)
        {
            int leftArmSeconds = getLeftArmSeconds();
            Functions.LeftArmRotateLeft(leftArmSeconds);
        }

        /// <summary>
        /// Rotate left arm right.
        /// </summary>
        private void LeftArmArmRotateRightBtn_Click(object sender, RoutedEventArgs e)
        {
            int leftArmSeconds = getLeftArmSeconds();
            Functions.LeftArmRotateRight(leftArmSeconds);
        }

        /// <summary>
        /// Move left arm up.
        /// </summary>
        private void LeftArmArmMoveUpBtn_Click(object sender, RoutedEventArgs e)
        {
            int leftArmSeconds = getLeftArmSeconds();
            Functions.LeftArmUp(leftArmSeconds);
        }

        /// <summary>
        /// Move left arm down.
        /// </summary>
        private void LeftArmArmMoveDownBtn_Click(object sender, RoutedEventArgs e)
        {
            int leftArmSeconds = getLeftArmSeconds();
            Functions.LeftArmDown(leftArmSeconds);
        }

        #endregion Left Arm Arm

        #region Left Arm Shake my hand

        /// <summary>
        /// Left arm shake my hand.
        /// </summary>
        private void LeftArmShakeMyHandBtn_Click(object sender, RoutedEventArgs e)
        {
            Functions.LeftHandshake();           
        }

        #endregion Left Arm Shake my hand

        #region Left Arm Stop

        /// <summary>
        /// Left arm stop.
        /// </summary>
        private void LeftArmStopBtn_Click(object sender, RoutedEventArgs e)
        {
            Functions.LeftArmStop();
        }

        #endregion Left Arm Stop

        #endregion Left Arm UI

        #region Left Arm Functions

        /// <summary>
        /// Returns the left arm seconds.
        /// </summary>
        /// <returns></returns>
        private int getLeftArmSeconds()
        {
            int seconds;
            Int32.TryParse(LeftArmSecondsTxtBox.Text, out seconds);
            return (seconds * 1000);
        }

        #endregion Left Arm Functions

        #endregion Left Arm

        #region Right Arm

        #region Right Arm UI

        #region Right Arm Lights

        /// <summary>
        /// Right arm lights on.
        /// </summary>
        private void RightEyeLight_Checked(object sender, RoutedEventArgs e)
        {
            Functions.RightLightsOn();
        }

        /// <summary>
        /// Right arm lights off.
        /// </summary>
        private void RightEyeLight_Unchecked(object sender, RoutedEventArgs e)
        {
            Functions.RightLightsOff();
        }

        #endregion Right Arm Lights

        #region Right Arm Claw

        /// <summary>
        /// Right arm claw open
        /// </summary>
        private void RightArmClawOpenBtn_Click(object sender, RoutedEventArgs e)
        {
            int rightArmSeconds = getRightArmSeconds();
            Functions.RightGrippersOpen(rightArmSeconds);
        }

        /// <summary>
        /// Right arm claw close.
        /// </summary>
        private void RightArmClawCloseBtn_Click(object sender, RoutedEventArgs e)
        {
            int rightArmSeconds = getRightArmSeconds();
            Functions.RightGrippersClose(rightArmSeconds);
        }

        /// <summary>
        /// Right arm claw up.
        /// </summary>
        private void RightArmClawMoveUpBtn_Click(object sender, RoutedEventArgs e)
        {
            int rightArmSeconds = getRightArmSeconds();
            Functions.RightGrippersUp(rightArmSeconds);
        }

        /// <summary>
        /// Right arm claw down.
        /// </summary>
        private void RightArmClawMoveDownBtn_Click(object sender, RoutedEventArgs e)
        {
            int rightArmSeconds = getRightArmSeconds();
            Functions.RightGrippersDown(rightArmSeconds);
        }

        #endregion Right Arm Claw

        #region Right Arm Elbow

        /// <summary>
        /// Right arm elbow up.
        /// </summary>
        private void RightArmElbowMoveUpBtn_Click(object sender, RoutedEventArgs e)
        {
            int rightArmSeconds = getRightArmSeconds();
            Functions.RightElbowUp(rightArmSeconds);
        }

        /// <summary>
        /// Right arm elbow down.
        /// </summary>
        private void RightArmElbowMoveDownBtn_Click(object sender, RoutedEventArgs e)
        {
            int rightArmSeconds = getRightArmSeconds();
            Functions.RightElbowDown(rightArmSeconds);
        }

        #endregion Right Arm Elbow

        #region Right Arm Arm

        /// <summary>
        /// Right arm rotate left. 
        /// </summary>
        private void RightArmArmRotateLeftBtn_Click(object sender, RoutedEventArgs e)
        {
            int rightArmSeconds = getRightArmSeconds();
            Functions.RightArmRotateLeft(rightArmSeconds);
        }

        /// <summary>
        /// Right arm rotate right.
        /// </summary>
        private void RightArmArmRotateRightBtn_Click(object sender, RoutedEventArgs e)
        {
            int rightArmSeconds = getRightArmSeconds();
            Functions.RightArmRotateRight(rightArmSeconds);
        }

        /// <summary>
        /// Right arm move up.
        /// </summary>
        private void RightArmArmMoveUpBtn_Click(object sender, RoutedEventArgs e)
        {
            int rightArmSeconds = getRightArmSeconds();
            Functions.RightArmUp(rightArmSeconds);
        }

        /// <summary>
        /// Right arm move down.
        /// </summary>
        private void RightArmArmMoveDownBtn_Click(object sender, RoutedEventArgs e)
        {
            int rightArmSeconds = getRightArmSeconds();
            Functions.RightArmDown(rightArmSeconds);
        }

        #endregion Right Arm Arm

        #region Right Arm Handshake

        /// <summary>
        /// Right arm shake my hand.
        /// </summary>
        private void RightArmShakeMyHandBtn_Click(object sender, RoutedEventArgs e)
        {
            Functions.RightHandshake();
        }

        #endregion Right Arm Shake my hand

        #region Right Arm Stop

        /// <summary>
        /// Right arm stop.
        /// </summary>
        private void RightArmStopBtn_Click(object sender, RoutedEventArgs e)
        {
            Functions.RightArmStop();
        }

        #endregion Right Arm Stop

        #endregion Right Arm UI

        #region Right Arm Functions

        /// <summary>
        /// Returns the right arm seconds.
        /// </summary>
        /// <returns></returns>
        private int getRightArmSeconds()
        {
            int seconds;
            Int32.TryParse(LeftArmSecondsTxtBox.Text, out seconds);
            return (seconds * 1000);
        }

        #endregion Right Arm Functions

        #endregion Right Arm

        #region iRobot Create

        #region iRobot Create UI

        #region iRobot Create Connections

        /// <summary>
        /// Connect iRobot Create.
        /// </summary>
        private void iRobotConnectSwitch_Checked(object sender, RoutedEventArgs e)
        {
            if (!Functions.irobot.IsOpen())
            {
                iRobotLogTextBox.Text = Functions.iRobotTryConnect(iRobotCOMportTxtBox.Text);
            }
        }

        /// <summary>
        /// Disconnect iRobot Create.
        /// </summary>
        private void iRobotConnectSwitch_Unchecked(object sender, RoutedEventArgs e)
        {
            if (Functions.irobot.IsOpen())
            {
                iRobotLogTextBox.Text = Functions.iRobotTryDisconnect();
            }
        }

        #endregion iRobot Create Connections

        #region iRobot Create Movement

        /// <summary>
        /// Move iRobot Create Forward.
        /// </summary>
        private void iRobotForwardBtn_Click(object sender, RoutedEventArgs e)
        {
            int iRobotVelocity = getiRobotVelocity();
            Functions.iRobotMoveForward(iRobotVelocity);
            //Functions.iRobotIncrementalMoveForward(iRobotVelocity);
        }

        /// <summary>
        /// Move iRobot Create Backward.
        /// </summary>
        private void iRobotBackwardBtn_Click(object sender, RoutedEventArgs e)
        {
            int iRobotVelocity = getiRobotVelocity();
            Functions.iRobotMoveBackward(iRobotVelocity);
            //Functions.iRobotIncrementalMoveBackward(iRobotVelocity);
        }

        /// <summary>
        /// Turn iRobot Create left.
        /// </summary>
        private void iRobotTurnLeftBtn_Click(object sender, RoutedEventArgs e)
        {
            int iRobotVelocity = getiRobotVelocity();
            Functions.iRobotTurnLeft(iRobotVelocity);
        }

        /// <summary>
        /// Turn iRobot Create right.
        /// </summary>
        private void iRobotTurnRightBtn_Click(object sender, RoutedEventArgs e)
        {
            int iRobotVelocity = getiRobotVelocity();
            Functions.iRobotTurnRight(iRobotVelocity);
        }

        /// <summary>
        /// Stop iRobot Create.
        /// </summary>
        private void iRobotStopBtn_Click(object sender, RoutedEventArgs e)
        {
            Functions.iRobotStop();
        }

        #endregion iRobot Create Movement

        #endregion iRobot Create UI

        #region iRobot Create Functions

        /// <summary>
        /// Get iRobot Create Velocity.
        /// </summary>
        /// <returns></returns>
        private int getiRobotVelocity()
        {
            int velocity;
            Int32.TryParse(VelocityTxtBox.Text, out velocity);
            return (velocity * 10);
        }

        #endregion iRobot Create Functions

        #endregion iRobot Create

        #region Kinect

        #region Vision

        private void KinectCameraModeSwitch_Checked(object sender, RoutedEventArgs e)
        {
            Functions.KinectActivateDepthMode();
        }

        private void KinectCameraModeSwitch_Unchecked(object sender, RoutedEventArgs e)
        {
            Functions.KinectActivateColorMode();
        }

        /// <summary>
        /// Handles the user clicking on the screenshot button
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void KinectScreenshotBtn_Click(object sender, RoutedEventArgs e)
        {
            Functions.KinectTakeScreenshot();
        }

        #endregion Vision

        #region Speech

        #region Speech Commands

        /// <summary>
        /// Handler for recognized speech events.
        /// </summary>
        /// <param name="sender">object sending the event.</param>
        /// <param name="e">event arguments.</param>
        private void SpeechRecognized(object sender, SpeechRecognizedEventArgs e)
        {
            const double ConfidenceThreshold = 0.7;
            if (e.Result.Confidence >= ConfidenceThreshold)
            {
                switch (e.Result.Semantics.Value.ToString())
                {
                    #region Left Arm Cases

                    case "LEFT GRIPPERS OPEN":
                        Functions.LeftGrippersOpen(1000);
                        break;
                    case "LEFT GRIPPERS CLOSE":
                        Functions.LeftGrippersClose(1000);
                        break;
                    case "LEFT LIGHTS ON":
                        Functions.LeftLightsOn();
                        break;
                    case "LEFT LIGHTS OFF":
                        Functions.LeftLightsOff();
                        break;
                    case "LEFT GRIPPERS UP":
                        Functions.LeftGrippersUp(1000);
                        break;
                    case "LEFT GRIPPERS DOWN":
                        Functions.LeftGrippersDown(1000);
                        break;
                    case "LEFT ELBOW UP":
                        Functions.LeftElbowUp(1000);
                        break;
                    case "LEFT ELBOW DOWN":
                        Functions.LeftElbowDown(1000);
                        break;
                    case "LEFT ARM UP":
                        Functions.LeftArmUp(1000);
                        break;
                    case "LEFT ARM DOWN":
                        Functions.LeftArmDown(1000);
                        break;
                    case "LEFT ARM LEFT":
                        Functions.LeftArmRotateLeft(1000);
                        break;
                    case "LEFT ARM RIGHT":
                        Functions.LeftArmRotateRight(1000);
                        break;
                    case "LEFT ARM STOP":
                        Functions.LeftArmStop();
                        break;
                    case "LEFT SHAKE MY HAND":
                        Functions.LeftHandshake();
                        break;

                    #endregion Left Arm Cases

                    #region Right Arm Cases

                    case "RIGHT GRIPPERS OPEN":
                        Functions.RightGrippersOpen(1000);
                        break;
                    case "RIGHT GRIPPERS CLOSE":
                        Functions.RightGrippersClose(1000);
                        break;
                    case "RIGHT LIGHTS ON":
                        Functions.RightLightsOn();
                        break;
                    case "RIGHT LIGHTS OFF":
                        Functions.RightLightsOff();
                        break;
                    case "RIGHT GRIPPERS UP":
                        Functions.RightGrippersUp(1000);
                        break;
                    case "RIGHT GRIPPERS DOWN":
                        Functions.RightGrippersDown(1000);
                        break;
                    case "RIGHT ELBOW UP":
                        Functions.RightElbowUp(1000);
                        break;
                    case "RIGHT ELBOW DOWN":
                        Functions.RightElbowDown(1000);
                        break;
                    case "RIGHT ARM UP":
                        Functions.RightArmUp(1000);
                        break;
                    case "RIGHT ARM DOWN":
                        Functions.RightArmDown(1000);
                        break;
                    case "RIGHT ARM LEFT":
                        Functions.RightArmRotateLeft(1000);
                        break;
                    case "RIGHT ARM RIGHT":
                        Functions.RightArmRotateRight(1000);
                        break;
                    case "RIGHT ARM STOP":
                        Functions.RightArmStop();
                        break;
                    case "RIGHT SHAKE MY HAND":
                        Functions.RightHandshake();
                        break;

                    #endregion Right Arm Cases

                    #region iRobot Create Cases

                    case "FORWARD":
                        if (Functions.irobot.IsOpen())
                        {
                            Functions.iRobotMoveForward(getiRobotVelocity());
                        }
                        break;
                    case "BACKWARD":
                        if (Functions.irobot.IsOpen())
                        {
                            Functions.iRobotMoveBackward(getiRobotVelocity());
                        }
                        break;
                    case "TURN LEFT":
                        if (Functions.irobot.IsOpen())
                        {
                            Functions.iRobotTurnLeft(getiRobotVelocity());
                        }
                        break;
                    case "TURN RIGHT":
                        if (Functions.irobot.IsOpen())
                        {
                            Functions.iRobotTurnRight(getiRobotVelocity());
                        }
                        break;
                    case "STOP CREATE":
                        if (Functions.irobot.IsOpen())
                        {
                            Functions.iRobotStop();
                        }
                        break;

                    #endregion iRobot Create Cases

                    #region Flash Eyes Cases

                    case "FLASH EYES":
                        Functions.FlashEyes(8000);
                        break;
                    case "STOP FLASH EYES":
                        // Missing
                        break;

                    #endregion Flash Eyes Cases

                    #region Dance Cases

                    case "ROBOT DANCE":
                        PlayMusic("sexy");
                        DanceThreadArray[0] = new Thread(() => Functions.Dance());
                        DanceThreadArray[0].Start();
                        //DanceThreadArray[0].Join();
                        //StopMusic();

                        break;
                    case "STOP DANCE":
                        DanceThreadArray[0].Abort();
                        Functions.LeftArmStop();
                        Functions.LeftLightsOff();
                        Functions.RightArmStop();
                        Functions.RightLightsOff();
                        Functions.iRobotStop();
                        StopMusic();
                        break;

                    #endregion Dance Cases

                    #region Follow Cases

                    case "FOLLOW ME":
                        FollowToggleSwitch.IsChecked = true;
                        Functions.FollowOn = true;
                        break;
                    case "STOP FOLLOW":
                        FollowToggleSwitch.IsChecked = false;
                        Functions.FollowOn = false;
                        FollowThreadArray[0] = new Thread(() => Functions.safeSlowdown());
                        FollowThreadArray[0].Start();
                        break;

                    #endregion Follow Cases

                    #region Converse Case

                    case "ROBOT":
                        //Process process;
                        //4 seconds to start 3 seconds to record 
                        //process = Process.Start("C:\\Users\\Kinect\\Desktop\\speech.exe");
                        //System.Threading.Thread.Sleep(4000);
                        //Functions.FlashEyes(8000);
                        break;

                    #endregion Converse Case

                    #region Kill Switch

                    case "STOP":
                        Functions.KillAllSwitch();
                        break;

                    #endregion Kill Switch
                }
            }
        }

        /// <summary>
        /// Handler for rejected speech events.
        /// </summary>
        /// <param name="sender">object sending the event.</param>
        /// <param name="e">event arguments.</param>
        private void SpeechRejected(object sender, SpeechRecognitionRejectedEventArgs e)
        {
            // what to do when a speech has been rejected.
        }

        #endregion Speech Commands

        #region Speech Recognition

        private void Log(string text)
        {
            lock (ConversationTxtBox)
            {
                ConversationTxtBox.Text += text + Environment.NewLine;
                ConversationTxtBox.SelectionStart = ConversationTxtBox.Text.Length;
            }
        }

        private void SpeakBtn_Click(object sender, RoutedEventArgs e)
        {
            //almost.....
            if (SpeechTxtBox.Text == "")
            {
                Console.WriteLine("No text");
            }
            else
            {
                Converse bot1 = new Converse(), bot2 = new Converse();
                System.Speech.Synthesis.SpeechSynthesizer speech1 = new System.Speech.Synthesis.SpeechSynthesizer(), speech2 = new System.Speech.Synthesis.SpeechSynthesizer();

                speech1.Rate = 0;
                speech2.Rate = 2;

                //insert speech 
                string message = SpeechTxtBox.Text;
                Log("Bot2: " + message);
                message = bot1.Think(message);

                Log("Bot1: " + message); speech1.Speak(message);
            }

        }

        private void SaveLogBtn_Click(object sender, RoutedEventArgs e)
        {
            // Save Log File

            SaveFileDialog saveDlg = new SaveFileDialog();
            saveDlg.Filter = "Text Files (*.txt)|*.txt";

            // Show save file dialog box. 
            Nullable<bool> result = saveDlg.ShowDialog();

            if (result == true)
            {
                using (StreamWriter streamWr = new StreamWriter(saveDlg.FileName, false, Encoding.UTF8))
                {
                    streamWr.Write(ConversationTxtBox.Text);
                }
            }
        }

        // NEEDS TO BE FIXED!!!!!!!!!!!!!!!!!
        private void ConverseBtn_Click(object sender, RoutedEventArgs e)
        {
            //Process process;
            ////4 seconds to start 3 seconds to record 
            //process = Process.Start("C:\\Users\\Kinect\\Desktop\\speech.exe");
            //System.Threading.Thread.Sleep(4000);
            Functions.FlashEyes(2000);
        }

        #endregion Speech Recognition

        private static RecognizerInfo GetKinectRecognizer()
        {
            foreach (RecognizerInfo recognizer in SpeechRecognitionEngine.InstalledRecognizers())
            {
                string value;
                recognizer.AdditionalInfo.TryGetValue("Kinect", out value);
                if ("True".Equals(value, StringComparison.OrdinalIgnoreCase) && "en-US".Equals(recognizer.Culture.Name, StringComparison.OrdinalIgnoreCase))
                {
                    return recognizer;
                }
            }

            return null;
        }

        #endregion Speech

        #region Kinect Tilt

        /// <summary>
        /// Tilts the Kinect Sensor up by x degrees, where x is the angle in int.
        /// </summary>
        /// <param name="angle">Tilt angle</param>
        private void KinectSensorUp(int angle)
        {
            if ((Functions.sensor.ElevationAngle + angle) <= Functions.sensor.MaxElevationAngle)
            {
                Functions.sensor.ElevationAngle += angle;
            }
            else
            {
                Functions.sensor.ElevationAngle = Functions.sensor.MaxElevationAngle;
            }
        }

        /// <summary>
        /// Tilts the Kinect Sensor down by x degrees, where x is the angle in int.
        /// </summary>
        /// <param name="angle">Tilt angle</param>
        private void KinectSensorDown(int angle)
        {
            if ((Functions.sensor.ElevationAngle - angle) >= Functions.sensor.MinElevationAngle)
            {
                Functions.sensor.ElevationAngle -= angle;
            }
            else
            {
                Functions.sensor.ElevationAngle = Functions.sensor.MinElevationAngle;
            }
        }

        private void KinectAngleSlider_PreviewMouseUp(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            Functions.KinectTiltAngle((int)KinectAngleSlider.Value);
        }

        #endregion Kinect Tilt

        #region Kinect UI

        /// <summary>
        /// Tilts the Kinect Sensor up by 5 degrees.
        /// </summary>
        private void KinectUpBtn_Click(object sender, RoutedEventArgs e)
        {
            Functions.KinectTiltUp(5);
        }

        /// <summary>
        /// Tilts the Kinect Sensor down by 5 degrees.
        /// </summary>
        private void KinectDownBtn_Click(object sender, RoutedEventArgs e)
        {
            Functions.KinectTiltDown(5);
        }

        /// <summary>
        /// Power the Kinect.
        /// </summary>
        private void KinectPowerSwitch_Checked(object sender, RoutedEventArgs e)
        {
            Functions.KinectPowerOn();
        }

        /// <summary>
        /// Power Off the Kinect.
        /// </summary>
        private void KinectPowerSwitch_Unchecked(object sender, RoutedEventArgs e)
        {
            Functions.KinectPowerOff();
        }

        #endregion Kinect UI

        #region Follow

        private void FollowToggleSwitch_Checked(object sender, RoutedEventArgs e)
        {
            //If the follow switch is turned off, turn it on and run follow
            Functions.FollowOn = true;
            Functions.followTimer.Start();
            Console.WriteLine("Timer Started");
            Functions.LeftLightsOn();
            Functions.RightLightsOn();
            //FollowThreadArray[0] = new Thread(() => Functions.Follow());
            //FollowThreadArray[0].Start();
            //Functions.iRobotMoveForward(300);
            //Functions.safeSpeedup(300);
            //Functions.iRobotCurrentSpeed = 300;
        }

        private void FollowToggleSwitch_Unchecked(object sender, RoutedEventArgs e)
        {
            //If the follow switch is turned on, turn it off and stop the robot
            Functions.FollowOn = false;
            Functions.LeftLightsOff();
            Functions.RightLightsOff();
           // int alex = Functions.iRobotCurrentSpeed;
            FollowThreadArray[0] = new Thread(() => Functions.safeSlowdown());
            FollowThreadArray[0].Start();
        }

        #endregion Follow

        #endregion Kinect

        #region Music

        private void PlayMusic(string songTitle)
        {
            // Get current directory.
            string currentDir = Environment.CurrentDirectory + "\\Music\\";

            switch (songTitle)
            {
                case "gangnam":
                    currentDir += "gangam.mp3";
                    MediaElement.Source = new Uri(currentDir);
                    break;
                case "Fame":
                    currentDir += "Fame.mp3";
                    MediaElement.Source = new Uri(currentDir);
                    break;
                case "DancingQueen":
                    currentDir += "DancingQueen.mp3";
                    MediaElement.Source = new Uri(currentDir);
                    break;
                case "lights":
                    currentDir += "Lights.mp3";
                    MediaElement.Source = new Uri(currentDir);
                    break;
                case "sexy":
                    currentDir += "Sexy.mp3";
                    MediaElement.Source = new Uri(currentDir);
                    break;
            }
            MediaElement.Play();
        }

        private void StopMusic()
        {
            MediaElement.Stop();
        }

        #endregion Music

        #region Kill All Switch

        private void KillAllSwitchBtn_Click(object sender, RoutedEventArgs e)
        {
            Functions.KillAllSwitch();
        }

        #endregion Kill All Switch

        
    }
}