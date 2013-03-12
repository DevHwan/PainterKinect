using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Kinect;

namespace PainterKinect
{
	/// <summary>
	/// MainWindow.xaml에 대한 상호 작용 논리
	/// </summary>
	public partial class MainWindow : Window
	{
		private KinectHandler kinectHandler;

		public MainWindow()
		{
			// Initialize Components
			InitializeComponent();

			// Initialize Logging Module
			Logging.InitializeLogging();

			// Set Kinect Handler
			this.kinectHandler = new KinectHandler();

			// Initialize Kinect
			if ( this.kinectHandler.InitializeKinectSensor() != KinectStatus.Connected )
			{
				// Close Window
				this.Close();
			}

			// Create Color Window
			ColorWindow colorWindow = new ColorWindow();
			colorWindow.Show();

			// Create Depth Window
			DepthImageWindow depthWindow = new DepthImageWindow();
			depthWindow.Show();

			// Log
			Logging.PrintLog( "MainWindow", "Program Initialized!" );
		}

		private void OnClosing( object sender, System.ComponentModel.CancelEventArgs e )
		{
			// Destroy Kinect
			this.kinectHandler.DestroyKinect();
			// Destroy Logging Module
			Logging.CloseLogging();
		}

		private void OnLoaded( object sender, RoutedEventArgs e )
		{

		}
	}
}
