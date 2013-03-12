using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Kinect;
using System.Windows.Media.Imaging;
using System.Windows.Media;
using System.Windows;

namespace PainterKinect
{
	public class KinectHandler
	{
		// Target Kinect Sensor
		private KinectSensor sensor;

		// Pixel Data Allocation
		private byte[] colorPixels;
		private DepthImagePixel[] depthPixels;
		private byte[] depthColorPixels;

		// Bitmap Container
		public WriteableBitmap colorBitmap;
		public WriteableBitmap depthBitmap;



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
				this.sensor.ColorStream.Enable( ColorImageFormat.RgbResolution640x480Fps30 );
				// Allocate Pixel Data Array
				this.colorPixels = new byte[this.sensor.ColorStream.FramePixelDataLength];
				// Set Writeable Bitmap
				this.colorBitmap = new WriteableBitmap( this.sensor.ColorStream.FrameWidth, this.sensor.ColorStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null );
				// Add Color Stream Event Handler
				this.sensor.ColorFrameReady += OnColorFrameReady;

				// Enable Depth Stream
				this.sensor.DepthStream.Enable( DepthImageFormat.Resolution640x480Fps30 );
				// Allocate Pixel Data Array
				this.depthPixels = new DepthImagePixel[this.sensor.DepthStream.FramePixelDataLength];
				// Allocate Pixel Depth Color Data Array
				this.depthColorPixels = new byte[this.sensor.DepthStream.FramePixelDataLength * sizeof( int )];
				// Set Writeable Bitmap
				this.depthBitmap = new WriteableBitmap( this.sensor.DepthStream.FrameWidth, this.sensor.DepthStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null );
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
					// Print Error Message
					Logging.PrintErrorLog( "InitializeKinectSensor", "Failed To Start Kinect Device!!" );
				}
			}

			// Return
			return retVal;
		}

		public void DestroyKinect()
		{
			if ( this.sensor != null )
			{
				this.sensor.Stop();
			}
		}

		private void OnColorFrameReady( object sender, ColorImageFrameReadyEventArgs e )
		{
			using ( ColorImageFrame colorFrame = e.OpenColorImageFrame() )
			{
				if ( colorFrame != null )
				{
					// Copy Pixel Data
					colorFrame.CopyPixelDataTo( this.colorPixels );

					// Write As Bitmap
					this.colorBitmap.WritePixels( new Int32Rect( 0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight ), this.colorPixels, this.colorBitmap.PixelWidth * sizeof( int ), 0 );
				}
			}
		}

		private void OnDepthFrameReady( object sender, DepthImageFrameReadyEventArgs e )
		{
			using ( DepthImageFrame depthFrame = e.OpenDepthImageFrame() )
			{
				if ( depthFrame != null )
				{
					// Copy Depth Pixel Data
					depthFrame.CopyDepthImagePixelDataTo( this.depthPixels );

					// Get Reliable Min Max Value
					int minDepth = depthFrame.MinDepth;
					int maxDepth = 2000;// depthFrame.MaxDepth;

					// Convert Depth to RGB
					int colorPixelIndex = 0;
					for ( int i = 0 ; i < this.depthPixels.Length ; i++ )
					{
						// Get Depth Value For This Pixel
						short depthValue = depthPixels[i].Depth;

						// Convert Intensity To Byte
						// TODO Make A lookup Table for Performance
						depthValue = (short)( depthValue >= minDepth && depthValue <= maxDepth ? depthValue : 0 );
						byte intensity = (byte)( depthValue * 255 / ( maxDepth - minDepth ) );

						// BGR
						this.depthColorPixels[colorPixelIndex++] = intensity;
						this.depthColorPixels[colorPixelIndex++] = intensity;
						this.depthColorPixels[colorPixelIndex++] = intensity;
						// Skip A
						colorPixelIndex++;
					}

					// Write Pixel Data As Bitmap
					this.depthBitmap.WritePixels( new Int32Rect( 0, 0, this.depthBitmap.PixelWidth, this.depthBitmap.PixelHeight ), this.depthColorPixels, this.depthBitmap.PixelWidth * sizeof( int ), 0 );
				}
			}
		}

		private void OnSkeletonFrameReady( object sender, SkeletonFrameReadyEventArgs e )
		{
			Skeleton[] skeletons;

			using ( SkeletonFrame skeletonFrame = e.OpenSkeletonFrame() )
			{
				if ( skeletonFrame != null )
				{
					// Allocate Skeleton Frames
					skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
					// Copy Skeleton Datas
					skeletonFrame.CopySkeletonDataTo( skeletons );
				}
			}


		}
	}
}
