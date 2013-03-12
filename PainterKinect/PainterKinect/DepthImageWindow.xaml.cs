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
using System.Windows.Shapes;

namespace PainterKinect
{
	/// <summary>
	/// DepthImageWindow.xaml에 대한 상호 작용 논리
	/// </summary>
	public partial class DepthImageWindow : Window
	{
		private KinectHandler kinectHandler;

		public DepthImageWindow( KinectHandler kinectHandler )
		{
			InitializeComponent();
			this.kinectHandler = kinectHandler;
		}

		private void OnMainWindowLoaded( object sender, RoutedEventArgs e )
		{
			this.DepthImageControl.Source = this.kinectHandler.depthBitmap;
		}
	}
}
