using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Kinect;

namespace PainterKinect
{
	class KinectHandler
	{
		// Target Kinect Sensor
		private KinectSensor sensor;

		public KinectHandler()
		{

		}

		public KinectStatus InitializeKinectSensor()
		{
			// Initial State
			KinectStatus retVal = KinectStatus.Undefined;

			// Find Available Kinect Sensor
			foreach ( var detectedSensors in KinectSensor.KinectSensors )
			{
				if ( detectedSensors.Status == KinectStatus.Connected )
				{
					this.sensor = detectedSensors;
					retVal = detectedSensors.Status;
				}
			}

			if ( this.sensor != null )
			{
				// Enable Color Stream
				this.sensor.ColorStream.Enable();
				// Add Color Stream Event Handler
				this.sensor.ColorFrameReady += OnColorFrameReady;

				// Enable Depth Stream
				this.sensor.DepthStream.Enable();
				// Add Depth Stream Event Handler
				this.sensor.DepthFrameReady += OnDepthFrameReady;

				// Enable Skeleton Stream
				this.sensor.SkeletonStream.Enable();
				// Add Skeleton Stream Event Handler
				this.sensor.SkeletonFrameReady += OnSkeletonFrameReady;

				// Try To Start Device
				try
				{
					this.sensor.Start();
				}
				catch (System.Exception ex)
				{
					// Set Null
					this.sensor = null;
					// Set Return Value
					retVal = KinectStatus.Error;
				}
			}

			// Return
			return retVal;
		}

		private void OnColorFrameReady( object sender, ColorImageFrameReadyEventArgs e )
		{

		}

		private void OnDepthFrameReady( object sender, DepthImageFrameReadyEventArgs e )
		{

		}

		private void OnSkeletonFrameReady( object sender, SkeletonFrameReadyEventArgs e )
		{

		}
	}
}
