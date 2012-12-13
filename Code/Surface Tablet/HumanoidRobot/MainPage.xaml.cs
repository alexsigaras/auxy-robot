using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Media.Imaging;
using Windows.UI.Xaml.Navigation;
using HumanoidRobot.HumanoidServiceReference;
using System.ServiceModel;
using System.ServiceModel.Channels;
using System.Runtime.InteropServices.WindowsRuntime;

// The Blank Page item template is documented at http://go.microsoft.com/fwlink/?LinkId=234238

namespace HumanoidRobot
{
    /// <summary>
    /// An empty page that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainPage : Page
    {
        //HumanoidServiceClient client = new HumanoidServiceClient(new BasicHttpBinding() { MaxBufferPoolSize = 0, MaxReceivedMessageSize = int.MaxValue, MaxBufferSize = int.MaxValue }, new EndpointAddress("http://Kinect-PC:31337/HumanoidService/endpoint"));
        HumanoidServiceClient client = new HumanoidServiceClient(new BasicHttpBinding() { MaxBufferPoolSize = 0, MaxReceivedMessageSize = 3100000, MaxBufferSize = 3100000 }, new EndpointAddress("http://Kinect-PC:31337/HumanoidService/endpoint"));

        public MainPage()
        {
            this.InitializeComponent();
        }

        /// <summary>
        /// Invoked when this page is about to be displayed in a Frame.
        /// </summary>
        /// <param name="e">Event data that describes how this page was reached.  The Parameter
        /// property is typically used to configure the page.</param>
        protected override void OnNavigatedTo(NavigationEventArgs e) {}

        private async void Page_Loaded(object sender, RoutedEventArgs e)
        {
         
            //var timer = new DispatcherTimer();
            //timer.Interval = TimeSpan.FromMilliseconds(75);
            
            //timer.Tick += async delegate
            //{
            //    //await UpdateKinectImage();
            //};
            //timer.Start();

            await UpdateKinectImage();
            await client.KinectTiltAngleAsync(-27);
        }

        private async System.Threading.Tasks.Task UpdateKinectImage()
        {
            var KinectStuff = await client.KinectStuffAsync();
            Windows.UI.Xaml.Media.Imaging.WriteableBitmap KinectWriteableBitmap = new Windows.UI.Xaml.Media.Imaging.WriteableBitmap(KinectStuff.pixelWidth, KinectStuff.pixelHeight);
            // Open a stream to copy the image contents to the WriteableBitmap's pixel buffer 
            using (Stream stream = KinectWriteableBitmap.PixelBuffer.AsStream())
            {
                byte[] decompressedColorPixels = Compressor.Decompress(KinectStuff.colorPixels);
                await stream.WriteAsync(decompressedColorPixels, 0, decompressedColorPixels.Length);
            }
            this.KinectImage.Source = KinectWriteableBitmap;
        }
            
        #region Left Arm

        private void LeftGrippersOpenBtn_Click(object sender, RoutedEventArgs e)
        {
            client.LeftGrippersOpenAsync(1);
        }

        private void LeftGrippersClose_Click(object sender, RoutedEventArgs e)
        {
            client.LeftGrippersCloseAsync(1);
        }

        private void LeftGrippersUpBtn_Click(object sender, RoutedEventArgs e)
        {
            client.LeftGrippersUpAsync(1);
        }

        private void LeftGrippersDownBtn_Click(object sender, RoutedEventArgs e)
        {
            client.LeftGrippersDownAsync(1);
        }

        private void LeftElbowUpBtn_Click(object sender, RoutedEventArgs e)
        {
            client.LeftElbowUpAsync(1);
        }

        private void LeftElbowDownBtn_Click(object sender, RoutedEventArgs e)
        {
            client.LeftElbowDownAsync(1);
        }

        private void LeftArmUpBtn_Click(object sender, RoutedEventArgs e)
        {
            client.LeftArmUpAsync(1);
        }

        private void LeftArmDownBtn_Click(object sender, RoutedEventArgs e)
        {
            client.LeftArmDownAsync(1);
        }

        private void LeftArmRotateLeftBtn_Click(object sender, RoutedEventArgs e)
        {
            client.LeftArmRotateLeftAsync(1);
        }

        private void LeftArmRotateRightBtn_Click(object sender, RoutedEventArgs e)
        {
            client.LeftArmRotateRightAsync(1);
        }

        private void LeftEyeSwitch_Toggled_1(object sender, RoutedEventArgs e)
        {
            if (((ToggleSwitch)sender).IsOn)
            {
                client.LeftLightsOnAsync();
            }
            else
            {
                client.LeftLightsOffAsync();
            }
        }

        private void LeftHandshakeBtn_Click(object sender, RoutedEventArgs e)
        {
            client.LeftHandshakeAsync();
        }

        #endregion Left Arm

        #region Right Arm

        private void RightGrippersOpenBtn_Click(object sender, RoutedEventArgs e)
        {
            client.RightGrippersOpenAsync(1);
        }

        private void RightGrippersClose_Click(object sender, RoutedEventArgs e)
        {
            client.RightGrippersCloseAsync(1);
        }

        private void RightGrippersUpBtn_Click(object sender, RoutedEventArgs e)
        {
            client.RightGrippersUpAsync(1);
        }

        private void RightEyeSwitch_Toggled_1(object sender, RoutedEventArgs e)
        {
            if (((ToggleSwitch)sender).IsOn)
            {
                client.RightLightsOnAsync();
            }
            else
            {
                client.RightLightsOffAsync();
            }
        }

        private void RightGrippersDownBtn_Click(object sender, RoutedEventArgs e)
        {
            client.RightGrippersDownAsync(1);
        }

        private void RightElbowUpBtn_Click(object sender, RoutedEventArgs e)
        {
            client.RightElbowUpAsync(1);
        }

        private void RightElbowDownBtn_Click(object sender, RoutedEventArgs e)
        {
            client.RightElbowDownAsync(1);
        }

        private void RightArmUpBtn_Click(object sender, RoutedEventArgs e)
        {
            client.RightArmUpAsync(1);
        }

        private void RightArmDownBtn_Click(object sender, RoutedEventArgs e)
        {
            client.RightArmDownAsync(1);
        }

        private void RightArmRotateLeftBtn_Click(object sender, RoutedEventArgs e)
        {
            client.RightArmRotateLeftAsync(1);
        }

        private void RightArmRotateRightBtn_Click(object sender, RoutedEventArgs e)
        {
            client.RightArmRotateRightAsync(1);
        }

        private void RightHandshakeBtn_Click(object sender, RoutedEventArgs e)
        {
            client.RightHandshakeAsync();
        }

        #endregion Right Arm

        #region Flash Eyes

        private void FlashEyesBtn_Click(object sender, RoutedEventArgs e)
        {
            client.FlashEyesAsync(8000);
        }

        #endregion Flash Eyes

        #region iRobot Create

        private void iCreateMoveForwardBtn_Click(object sender, RoutedEventArgs e)
        {
            client.iRobotMoveForwardAsync(50);
        }

        private void iCreateStopBtn_Click(object sender, RoutedEventArgs e)
        {
            client.iRobotStopAsync();
        }

        private void iCreateMoveBackwardBtn_Click(object sender, RoutedEventArgs e)
        {
            client.iRobotMoveBackwardAsync(50);
        }

        private void iCreateTurnLeftBtn_Click(object sender, RoutedEventArgs e)
        {
            client.iRobotTurnLeftAsync(50);
        }

        private void iCreateTurnRightBtn_Click(object sender, RoutedEventArgs e)
        {
            client.iRobotTurnRightAsync(50);
        }

        private void iRobotConnectSwitch_Toggled(object sender, RoutedEventArgs e)
        {
            if (((ToggleSwitch)sender).IsOn)
            {
                if (COMPortComboBox != null)
                {
                    client.iRobotConnectAsync(COMPortComboBox.SelectedValue.ToString());
                }
            }
            else
            {
                client.iRobotDisconnectAsync();
            }
        }

        #endregion iRobot Create

        #region Kinect

        private void KinectTiltSlider_PointerCaptureLost(object sender, PointerRoutedEventArgs e)
        {
            client.KinectTiltAngleAsync((int)KinectTiltSlider.Value);
        } 

        private void KinectPowerSwitch_Toggled(object sender, RoutedEventArgs e)
        {
            if (((ToggleSwitch)sender).IsOn)
            {
                client.KinectPowerOnAsync();
            }
            else
            {
                client.KinectPowerOffAsync();
            }
        }

        private void KinectCameraModeSwitch_Toggled(object sender, RoutedEventArgs e)
        {
            if (((ToggleSwitch)sender).IsOn)
            {
                client.KinectActivateDepthModeAsync();
            }
            else
            {
                client.KinectActivateColorModeAsync();
            }
        }

        #endregion Kinect

        #region Music

        private void PlayMusic(string songTitle)
        {
            // Get current directory.

            var folder = Windows.ApplicationModel.Package.Current.InstalledLocation.Path;

            string currentDir = folder.ToString() + "\\Assets\\Music\\";
            switch (songTitle)
            {
                case "Gangnam Style":
                    currentDir += "gangnam.mp3";
                    MediaElement.Source = new Uri(currentDir);
                    break;
                case "Fame":
                    currentDir += "Fame.mp3";
                    MediaElement.Source = new Uri(currentDir);
                    break;
                case "Dancing Queen":
                    currentDir += "DancingQueen.mp3";
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

        private void FollowSwitch_Toggled(object sender, RoutedEventArgs e)
        {
            if (((ToggleSwitch)sender).IsOn)
            {
                client.FollowAsync(true);
            }
            else
            {
                client.FollowAsync(false);
            }
        }

        #region Dance

        private void DanceBtn_Click(object sender, RoutedEventArgs e)
        {
            string songTitle = DanceSongComboBox.SelectedValue.ToString();
            PlayMusic(songTitle);
            client.DanceAsync();
        }

        #endregion Dance

        #region Kill All Switch

        private async void StopBtn_Click(object sender, RoutedEventArgs e)
        {
            await client.KillAllSwitchAsync();
        }

        #endregion Kill All Switch

        private async void Button_Click(object sender, RoutedEventArgs e)
        {
            await UpdateKinectImage();
        }

    }
}
