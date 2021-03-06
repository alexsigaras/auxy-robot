﻿//------------------------------------------------------------------------------
// <auto-generated>
//     This code was generated by a tool.
//     Runtime Version:4.0.30319.18010
//
//     Changes to this file may cause incorrect behavior and will be lost if
//     the code is regenerated.
// </auto-generated>
//------------------------------------------------------------------------------

// 
// This code was auto-generated by Microsoft.VisualStudio.ServiceReference.Platforms, version 11.0.50727.1
// 
namespace HumanoidRobot.HumanoidServiceReference {
    using System.Runtime.Serialization;
    
    
    [System.Diagnostics.DebuggerStepThroughAttribute()]
    [System.CodeDom.Compiler.GeneratedCodeAttribute("System.Runtime.Serialization", "4.0.0.0")]
    [System.Runtime.Serialization.DataContractAttribute(Name="HumanoidService.PixelData", Namespace="http://schemas.datacontract.org/2004/07/HumanoidRobot")]
    public partial class HumanoidServicePixelData : object, System.ComponentModel.INotifyPropertyChanged {
        
        private byte[] colorPixelsField;
        
        private int pixelHeightField;
        
        private int pixelWidthField;
        
        [System.Runtime.Serialization.DataMemberAttribute()]
        public byte[] colorPixels {
            get {
                return this.colorPixelsField;
            }
            set {
                if ((object.ReferenceEquals(this.colorPixelsField, value) != true)) {
                    this.colorPixelsField = value;
                    this.RaisePropertyChanged("colorPixels");
                }
            }
        }
        
        [System.Runtime.Serialization.DataMemberAttribute()]
        public int pixelHeight {
            get {
                return this.pixelHeightField;
            }
            set {
                if ((this.pixelHeightField.Equals(value) != true)) {
                    this.pixelHeightField = value;
                    this.RaisePropertyChanged("pixelHeight");
                }
            }
        }
        
        [System.Runtime.Serialization.DataMemberAttribute()]
        public int pixelWidth {
            get {
                return this.pixelWidthField;
            }
            set {
                if ((this.pixelWidthField.Equals(value) != true)) {
                    this.pixelWidthField = value;
                    this.RaisePropertyChanged("pixelWidth");
                }
            }
        }
        
        public event System.ComponentModel.PropertyChangedEventHandler PropertyChanged;
        
        protected void RaisePropertyChanged(string propertyName) {
            System.ComponentModel.PropertyChangedEventHandler propertyChanged = this.PropertyChanged;
            if ((propertyChanged != null)) {
                propertyChanged(this, new System.ComponentModel.PropertyChangedEventArgs(propertyName));
            }
        }
    }
    
    [System.CodeDom.Compiler.GeneratedCodeAttribute("System.ServiceModel", "4.0.0.0")]
    [System.ServiceModel.ServiceContractAttribute(ConfigurationName="HumanoidServiceReference.IHumanoidService")]
    public interface IHumanoidService {
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/RightArmRotateRight", ReplyAction="http://tempuri.org/IHumanoidService/RightArmRotateRightResponse")]
        System.Threading.Tasks.Task RightArmRotateRightAsync(int seconds);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/RightHandshake", ReplyAction="http://tempuri.org/IHumanoidService/RightHandshakeResponse")]
        System.Threading.Tasks.Task RightHandshakeAsync();
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/RightArmStop", ReplyAction="http://tempuri.org/IHumanoidService/RightArmStopResponse")]
        System.Threading.Tasks.Task RightArmStopAsync();
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/iRobotConnect", ReplyAction="http://tempuri.org/IHumanoidService/iRobotConnectResponse")]
        System.Threading.Tasks.Task<string> iRobotConnectAsync(string COMPort);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/iRobotDisconnect", ReplyAction="http://tempuri.org/IHumanoidService/iRobotDisconnectResponse")]
        System.Threading.Tasks.Task<string> iRobotDisconnectAsync();
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/iRobotMoveForward", ReplyAction="http://tempuri.org/IHumanoidService/iRobotMoveForwardResponse")]
        System.Threading.Tasks.Task iRobotMoveForwardAsync(int iRobotVelocity);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/iRobotMoveBackward", ReplyAction="http://tempuri.org/IHumanoidService/iRobotMoveBackwardResponse")]
        System.Threading.Tasks.Task iRobotMoveBackwardAsync(int iRobotVelocity);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/iRobotTurnLeft", ReplyAction="http://tempuri.org/IHumanoidService/iRobotTurnLeftResponse")]
        System.Threading.Tasks.Task iRobotTurnLeftAsync(int iRobotVelocity);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/iRobotTurnRight", ReplyAction="http://tempuri.org/IHumanoidService/iRobotTurnRightResponse")]
        System.Threading.Tasks.Task iRobotTurnRightAsync(int iRobotVelocity);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/iRobotStop", ReplyAction="http://tempuri.org/IHumanoidService/iRobotStopResponse")]
        System.Threading.Tasks.Task iRobotStopAsync();
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/KinectPowerOn", ReplyAction="http://tempuri.org/IHumanoidService/KinectPowerOnResponse")]
        System.Threading.Tasks.Task KinectPowerOnAsync();
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/KinectPowerOff", ReplyAction="http://tempuri.org/IHumanoidService/KinectPowerOffResponse")]
        System.Threading.Tasks.Task KinectPowerOffAsync();
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/KinectTiltUp", ReplyAction="http://tempuri.org/IHumanoidService/KinectTiltUpResponse")]
        System.Threading.Tasks.Task KinectTiltUpAsync(int angle);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/KinectTiltDown", ReplyAction="http://tempuri.org/IHumanoidService/KinectTiltDownResponse")]
        System.Threading.Tasks.Task KinectTiltDownAsync(int angle);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/KinectTiltAngle", ReplyAction="http://tempuri.org/IHumanoidService/KinectTiltAngleResponse")]
        System.Threading.Tasks.Task KinectTiltAngleAsync(int angle);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/KinectActivateDepthMode", ReplyAction="http://tempuri.org/IHumanoidService/KinectActivateDepthModeResponse")]
        System.Threading.Tasks.Task KinectActivateDepthModeAsync();
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/KinectActivateColorMode", ReplyAction="http://tempuri.org/IHumanoidService/KinectActivateColorModeResponse")]
        System.Threading.Tasks.Task KinectActivateColorModeAsync();
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/KinectTakeScreenshot", ReplyAction="http://tempuri.org/IHumanoidService/KinectTakeScreenshotResponse")]
        System.Threading.Tasks.Task KinectTakeScreenshotAsync();
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/KinectStuff", ReplyAction="http://tempuri.org/IHumanoidService/KinectStuffResponse")]
        System.Threading.Tasks.Task<HumanoidRobot.HumanoidServiceReference.HumanoidServicePixelData> KinectStuffAsync();
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/FlashEyes", ReplyAction="http://tempuri.org/IHumanoidService/FlashEyesResponse")]
        System.Threading.Tasks.Task FlashEyesAsync(int times);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/Dance", ReplyAction="http://tempuri.org/IHumanoidService/DanceResponse")]
        System.Threading.Tasks.Task DanceAsync();
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/Follow", ReplyAction="http://tempuri.org/IHumanoidService/FollowResponse")]
        System.Threading.Tasks.Task FollowAsync(bool FollowOn);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/KillAllSwitch", ReplyAction="http://tempuri.org/IHumanoidService/KillAllSwitchResponse")]
        System.Threading.Tasks.Task KillAllSwitchAsync();
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/LeftLightsOn", ReplyAction="http://tempuri.org/IHumanoidService/LeftLightsOnResponse")]
        System.Threading.Tasks.Task LeftLightsOnAsync();
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/LeftLightsOff", ReplyAction="http://tempuri.org/IHumanoidService/LeftLightsOffResponse")]
        System.Threading.Tasks.Task LeftLightsOffAsync();
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/LeftGrippersOpen", ReplyAction="http://tempuri.org/IHumanoidService/LeftGrippersOpenResponse")]
        System.Threading.Tasks.Task LeftGrippersOpenAsync(int leftArmSeconds);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/LeftGrippersClose", ReplyAction="http://tempuri.org/IHumanoidService/LeftGrippersCloseResponse")]
        System.Threading.Tasks.Task LeftGrippersCloseAsync(int leftArmSeconds);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/LeftGrippersUp", ReplyAction="http://tempuri.org/IHumanoidService/LeftGrippersUpResponse")]
        System.Threading.Tasks.Task LeftGrippersUpAsync(int leftArmSeconds);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/LeftGrippersDown", ReplyAction="http://tempuri.org/IHumanoidService/LeftGrippersDownResponse")]
        System.Threading.Tasks.Task LeftGrippersDownAsync(int leftArmSeconds);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/LeftElbowUp", ReplyAction="http://tempuri.org/IHumanoidService/LeftElbowUpResponse")]
        System.Threading.Tasks.Task LeftElbowUpAsync(int leftArmSeconds);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/LeftElbowDown", ReplyAction="http://tempuri.org/IHumanoidService/LeftElbowDownResponse")]
        System.Threading.Tasks.Task LeftElbowDownAsync(int leftArmSeconds);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/LeftArmUp", ReplyAction="http://tempuri.org/IHumanoidService/LeftArmUpResponse")]
        System.Threading.Tasks.Task LeftArmUpAsync(int leftArmSeconds);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/LeftArmDown", ReplyAction="http://tempuri.org/IHumanoidService/LeftArmDownResponse")]
        System.Threading.Tasks.Task LeftArmDownAsync(int leftArmSeconds);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/LeftArmRotateLeft", ReplyAction="http://tempuri.org/IHumanoidService/LeftArmRotateLeftResponse")]
        System.Threading.Tasks.Task LeftArmRotateLeftAsync(int leftArmSeconds);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/LeftArmRotateRight", ReplyAction="http://tempuri.org/IHumanoidService/LeftArmRotateRightResponse")]
        System.Threading.Tasks.Task LeftArmRotateRightAsync(int leftArmSeconds);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/LeftHandshake", ReplyAction="http://tempuri.org/IHumanoidService/LeftHandshakeResponse")]
        System.Threading.Tasks.Task LeftHandshakeAsync();
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/LeftArmStop", ReplyAction="http://tempuri.org/IHumanoidService/LeftArmStopResponse")]
        System.Threading.Tasks.Task LeftArmStopAsync();
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/RightLightsOn", ReplyAction="http://tempuri.org/IHumanoidService/RightLightsOnResponse")]
        System.Threading.Tasks.Task RightLightsOnAsync();
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/RightLightsOff", ReplyAction="http://tempuri.org/IHumanoidService/RightLightsOffResponse")]
        System.Threading.Tasks.Task RightLightsOffAsync();
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/RightGrippersOpen", ReplyAction="http://tempuri.org/IHumanoidService/RightGrippersOpenResponse")]
        System.Threading.Tasks.Task RightGrippersOpenAsync(int seconds);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/RightGrippersClose", ReplyAction="http://tempuri.org/IHumanoidService/RightGrippersCloseResponse")]
        System.Threading.Tasks.Task RightGrippersCloseAsync(int seconds);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/RightGrippersUp", ReplyAction="http://tempuri.org/IHumanoidService/RightGrippersUpResponse")]
        System.Threading.Tasks.Task RightGrippersUpAsync(int seconds);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/RightGrippersDown", ReplyAction="http://tempuri.org/IHumanoidService/RightGrippersDownResponse")]
        System.Threading.Tasks.Task RightGrippersDownAsync(int seconds);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/RightElbowUp", ReplyAction="http://tempuri.org/IHumanoidService/RightElbowUpResponse")]
        System.Threading.Tasks.Task RightElbowUpAsync(int seconds);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/RightElbowDown", ReplyAction="http://tempuri.org/IHumanoidService/RightElbowDownResponse")]
        System.Threading.Tasks.Task RightElbowDownAsync(int seconds);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/RightArmUp", ReplyAction="http://tempuri.org/IHumanoidService/RightArmUpResponse")]
        System.Threading.Tasks.Task RightArmUpAsync(int seconds);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/RightArmDown", ReplyAction="http://tempuri.org/IHumanoidService/RightArmDownResponse")]
        System.Threading.Tasks.Task RightArmDownAsync(int seconds);
        
        [System.ServiceModel.OperationContractAttribute(Action="http://tempuri.org/IHumanoidService/RightArmRotateLeft", ReplyAction="http://tempuri.org/IHumanoidService/RightArmRotateLeftResponse")]
        System.Threading.Tasks.Task RightArmRotateLeftAsync(int seconds);
    }
    
    [System.CodeDom.Compiler.GeneratedCodeAttribute("System.ServiceModel", "4.0.0.0")]
    public interface IHumanoidServiceChannel : HumanoidRobot.HumanoidServiceReference.IHumanoidService, System.ServiceModel.IClientChannel {
    }
    
    [System.Diagnostics.DebuggerStepThroughAttribute()]
    [System.CodeDom.Compiler.GeneratedCodeAttribute("System.ServiceModel", "4.0.0.0")]
    public partial class HumanoidServiceClient : System.ServiceModel.ClientBase<HumanoidRobot.HumanoidServiceReference.IHumanoidService>, HumanoidRobot.HumanoidServiceReference.IHumanoidService {
        
        /// <summary>
        /// Implement this partial method to configure the service endpoint.
        /// </summary>
        /// <param name="serviceEndpoint">The endpoint to configure</param>
        /// <param name="clientCredentials">The client credentials</param>
        static partial void ConfigureEndpoint(System.ServiceModel.Description.ServiceEndpoint serviceEndpoint, System.ServiceModel.Description.ClientCredentials clientCredentials);
        
        public HumanoidServiceClient() : 
                base(HumanoidServiceClient.GetDefaultBinding(), HumanoidServiceClient.GetDefaultEndpointAddress()) {
            this.Endpoint.Name = EndpointConfiguration.BasicHttpBinding_IHumanoidService.ToString();
            ConfigureEndpoint(this.Endpoint, this.ClientCredentials);
        }
        
        public HumanoidServiceClient(EndpointConfiguration endpointConfiguration) : 
                base(HumanoidServiceClient.GetBindingForEndpoint(endpointConfiguration), HumanoidServiceClient.GetEndpointAddress(endpointConfiguration)) {
            this.Endpoint.Name = endpointConfiguration.ToString();
            ConfigureEndpoint(this.Endpoint, this.ClientCredentials);
        }
        
        public HumanoidServiceClient(EndpointConfiguration endpointConfiguration, string remoteAddress) : 
                base(HumanoidServiceClient.GetBindingForEndpoint(endpointConfiguration), new System.ServiceModel.EndpointAddress(remoteAddress)) {
            this.Endpoint.Name = endpointConfiguration.ToString();
            ConfigureEndpoint(this.Endpoint, this.ClientCredentials);
        }
        
        public HumanoidServiceClient(EndpointConfiguration endpointConfiguration, System.ServiceModel.EndpointAddress remoteAddress) : 
                base(HumanoidServiceClient.GetBindingForEndpoint(endpointConfiguration), remoteAddress) {
            this.Endpoint.Name = endpointConfiguration.ToString();
            ConfigureEndpoint(this.Endpoint, this.ClientCredentials);
        }
        
        public HumanoidServiceClient(System.ServiceModel.Channels.Binding binding, System.ServiceModel.EndpointAddress remoteAddress) : 
                base(binding, remoteAddress) {
        }
        
        public System.Threading.Tasks.Task RightArmRotateRightAsync(int seconds) {
            return base.Channel.RightArmRotateRightAsync(seconds);
        }
        
        public System.Threading.Tasks.Task RightHandshakeAsync() {
            return base.Channel.RightHandshakeAsync();
        }
        
        public System.Threading.Tasks.Task RightArmStopAsync() {
            return base.Channel.RightArmStopAsync();
        }
        
        public System.Threading.Tasks.Task<string> iRobotConnectAsync(string COMPort) {
            return base.Channel.iRobotConnectAsync(COMPort);
        }
        
        public System.Threading.Tasks.Task<string> iRobotDisconnectAsync() {
            return base.Channel.iRobotDisconnectAsync();
        }
        
        public System.Threading.Tasks.Task iRobotMoveForwardAsync(int iRobotVelocity) {
            return base.Channel.iRobotMoveForwardAsync(iRobotVelocity);
        }
        
        public System.Threading.Tasks.Task iRobotMoveBackwardAsync(int iRobotVelocity) {
            return base.Channel.iRobotMoveBackwardAsync(iRobotVelocity);
        }
        
        public System.Threading.Tasks.Task iRobotTurnLeftAsync(int iRobotVelocity) {
            return base.Channel.iRobotTurnLeftAsync(iRobotVelocity);
        }
        
        public System.Threading.Tasks.Task iRobotTurnRightAsync(int iRobotVelocity) {
            return base.Channel.iRobotTurnRightAsync(iRobotVelocity);
        }
        
        public System.Threading.Tasks.Task iRobotStopAsync() {
            return base.Channel.iRobotStopAsync();
        }
        
        public System.Threading.Tasks.Task KinectPowerOnAsync() {
            return base.Channel.KinectPowerOnAsync();
        }
        
        public System.Threading.Tasks.Task KinectPowerOffAsync() {
            return base.Channel.KinectPowerOffAsync();
        }
        
        public System.Threading.Tasks.Task KinectTiltUpAsync(int angle) {
            return base.Channel.KinectTiltUpAsync(angle);
        }
        
        public System.Threading.Tasks.Task KinectTiltDownAsync(int angle) {
            return base.Channel.KinectTiltDownAsync(angle);
        }
        
        public System.Threading.Tasks.Task KinectTiltAngleAsync(int angle) {
            return base.Channel.KinectTiltAngleAsync(angle);
        }
        
        public System.Threading.Tasks.Task KinectActivateDepthModeAsync() {
            return base.Channel.KinectActivateDepthModeAsync();
        }
        
        public System.Threading.Tasks.Task KinectActivateColorModeAsync() {
            return base.Channel.KinectActivateColorModeAsync();
        }
        
        public System.Threading.Tasks.Task KinectTakeScreenshotAsync() {
            return base.Channel.KinectTakeScreenshotAsync();
        }
        
        public System.Threading.Tasks.Task<HumanoidRobot.HumanoidServiceReference.HumanoidServicePixelData> KinectStuffAsync() {
            return base.Channel.KinectStuffAsync();
        }
        
        public System.Threading.Tasks.Task FlashEyesAsync(int times) {
            return base.Channel.FlashEyesAsync(times);
        }
        
        public System.Threading.Tasks.Task DanceAsync() {
            return base.Channel.DanceAsync();
        }
        
        public System.Threading.Tasks.Task FollowAsync(bool FollowOn) {
            return base.Channel.FollowAsync(FollowOn);
        }
        
        public System.Threading.Tasks.Task KillAllSwitchAsync() {
            return base.Channel.KillAllSwitchAsync();
        }
        
        public System.Threading.Tasks.Task LeftLightsOnAsync() {
            return base.Channel.LeftLightsOnAsync();
        }
        
        public System.Threading.Tasks.Task LeftLightsOffAsync() {
            return base.Channel.LeftLightsOffAsync();
        }
        
        public System.Threading.Tasks.Task LeftGrippersOpenAsync(int leftArmSeconds) {
            return base.Channel.LeftGrippersOpenAsync(leftArmSeconds);
        }
        
        public System.Threading.Tasks.Task LeftGrippersCloseAsync(int leftArmSeconds) {
            return base.Channel.LeftGrippersCloseAsync(leftArmSeconds);
        }
        
        public System.Threading.Tasks.Task LeftGrippersUpAsync(int leftArmSeconds) {
            return base.Channel.LeftGrippersUpAsync(leftArmSeconds);
        }
        
        public System.Threading.Tasks.Task LeftGrippersDownAsync(int leftArmSeconds) {
            return base.Channel.LeftGrippersDownAsync(leftArmSeconds);
        }
        
        public System.Threading.Tasks.Task LeftElbowUpAsync(int leftArmSeconds) {
            return base.Channel.LeftElbowUpAsync(leftArmSeconds);
        }
        
        public System.Threading.Tasks.Task LeftElbowDownAsync(int leftArmSeconds) {
            return base.Channel.LeftElbowDownAsync(leftArmSeconds);
        }
        
        public System.Threading.Tasks.Task LeftArmUpAsync(int leftArmSeconds) {
            return base.Channel.LeftArmUpAsync(leftArmSeconds);
        }
        
        public System.Threading.Tasks.Task LeftArmDownAsync(int leftArmSeconds) {
            return base.Channel.LeftArmDownAsync(leftArmSeconds);
        }
        
        public System.Threading.Tasks.Task LeftArmRotateLeftAsync(int leftArmSeconds) {
            return base.Channel.LeftArmRotateLeftAsync(leftArmSeconds);
        }
        
        public System.Threading.Tasks.Task LeftArmRotateRightAsync(int leftArmSeconds) {
            return base.Channel.LeftArmRotateRightAsync(leftArmSeconds);
        }
        
        public System.Threading.Tasks.Task LeftHandshakeAsync() {
            return base.Channel.LeftHandshakeAsync();
        }
        
        public System.Threading.Tasks.Task LeftArmStopAsync() {
            return base.Channel.LeftArmStopAsync();
        }
        
        public System.Threading.Tasks.Task RightLightsOnAsync() {
            return base.Channel.RightLightsOnAsync();
        }
        
        public System.Threading.Tasks.Task RightLightsOffAsync() {
            return base.Channel.RightLightsOffAsync();
        }
        
        public System.Threading.Tasks.Task RightGrippersOpenAsync(int seconds) {
            return base.Channel.RightGrippersOpenAsync(seconds);
        }
        
        public System.Threading.Tasks.Task RightGrippersCloseAsync(int seconds) {
            return base.Channel.RightGrippersCloseAsync(seconds);
        }
        
        public System.Threading.Tasks.Task RightGrippersUpAsync(int seconds) {
            return base.Channel.RightGrippersUpAsync(seconds);
        }
        
        public System.Threading.Tasks.Task RightGrippersDownAsync(int seconds) {
            return base.Channel.RightGrippersDownAsync(seconds);
        }
        
        public System.Threading.Tasks.Task RightElbowUpAsync(int seconds) {
            return base.Channel.RightElbowUpAsync(seconds);
        }
        
        public System.Threading.Tasks.Task RightElbowDownAsync(int seconds) {
            return base.Channel.RightElbowDownAsync(seconds);
        }
        
        public System.Threading.Tasks.Task RightArmUpAsync(int seconds) {
            return base.Channel.RightArmUpAsync(seconds);
        }
        
        public System.Threading.Tasks.Task RightArmDownAsync(int seconds) {
            return base.Channel.RightArmDownAsync(seconds);
        }
        
        public System.Threading.Tasks.Task RightArmRotateLeftAsync(int seconds) {
            return base.Channel.RightArmRotateLeftAsync(seconds);
        }
        
        public virtual System.Threading.Tasks.Task OpenAsync() {
            return System.Threading.Tasks.Task.Factory.FromAsync(((System.ServiceModel.ICommunicationObject)(this)).BeginOpen(null, null), new System.Action<System.IAsyncResult>(((System.ServiceModel.ICommunicationObject)(this)).EndOpen));
        }
        
        public virtual System.Threading.Tasks.Task CloseAsync() {
            return System.Threading.Tasks.Task.Factory.FromAsync(((System.ServiceModel.ICommunicationObject)(this)).BeginClose(null, null), new System.Action<System.IAsyncResult>(((System.ServiceModel.ICommunicationObject)(this)).EndClose));
        }
        
        private static System.ServiceModel.Channels.Binding GetBindingForEndpoint(EndpointConfiguration endpointConfiguration) {
            if ((endpointConfiguration == EndpointConfiguration.BasicHttpBinding_IHumanoidService)) {
                System.ServiceModel.BasicHttpBinding result = new System.ServiceModel.BasicHttpBinding();
                result.MaxBufferSize = int.MaxValue;
                result.ReaderQuotas = System.Xml.XmlDictionaryReaderQuotas.Max;
                result.MaxReceivedMessageSize = int.MaxValue;
                result.AllowCookies = true;
                return result;
            }
            throw new System.InvalidOperationException(string.Format("Could not find endpoint with name \'{0}\'.", endpointConfiguration));
        }
        
        private static System.ServiceModel.EndpointAddress GetEndpointAddress(EndpointConfiguration endpointConfiguration) {
            if ((endpointConfiguration == EndpointConfiguration.BasicHttpBinding_IHumanoidService)) {
                return new System.ServiceModel.EndpointAddress("http://kinect-pc:31337/HumanoidService/endpoint");
            }
            throw new System.InvalidOperationException(string.Format("Could not find endpoint with name \'{0}\'.", endpointConfiguration));
        }
        
        private static System.ServiceModel.Channels.Binding GetDefaultBinding() {
            return HumanoidServiceClient.GetBindingForEndpoint(EndpointConfiguration.BasicHttpBinding_IHumanoidService);
        }
        
        private static System.ServiceModel.EndpointAddress GetDefaultEndpointAddress() {
            return HumanoidServiceClient.GetEndpointAddress(EndpointConfiguration.BasicHttpBinding_IHumanoidService);
        }
        
        public enum EndpointConfiguration {
            
            BasicHttpBinding_IHumanoidService,
        }
    }
}
