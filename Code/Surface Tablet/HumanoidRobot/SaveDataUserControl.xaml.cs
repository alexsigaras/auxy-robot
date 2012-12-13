using HumanoidRobot.Classes;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.Storage;
using Windows.UI.ApplicationSettings;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Navigation;

// The User Control item template is documented at http://go.microsoft.com/fwlink/?LinkId=234236

namespace HumanoidRobot
{
    public sealed partial class SaveDataUserControl : UserControl
    {
        public SaveDataUserControl()
        {
            this.InitializeComponent();
        }

        private void CloseBtn_Click(object sender, RoutedEventArgs e)
        {
            if (this.Parent.GetType() == typeof(Popup))
            {
                ((Popup)this.Parent).IsOpen = false;
            }
            SettingsPane.Show();
        }

        private void saveSwitch_Toggled_1(object sender, RoutedEventArgs e)
        {
            if (saveSwitch != null)
            {
                Settings.rememberState = saveSwitch.IsOn;
                ApplicationData.Current.LocalSettings.Values["rememberState"] = Settings.rememberState;
            }
        }
    }
}
