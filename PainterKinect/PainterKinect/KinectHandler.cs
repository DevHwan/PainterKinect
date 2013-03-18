using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Kinect;
using System.Windows.Media.Imaging;
using System.Windows.Media;
using System.Windows;
using OpenCvSharp;

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

		// Color & Depth Image
		private CvMat colorImageMat;
		private CvMat depthImageMat;

		// Hand Region Image
		private CvMat leftHandImageMat;
		private CvMat rightHandImageMat;

		// Hand Region Depth Image
		private CvMat leftHandDepthImageMat;
		private CvMat rightHandDepthImageMat;

		// Hand Tracked State
		private bool leftHandFound = false;
		private bool rightHandFound = false;

		// Skeleton Container
		public Skeleton[] skeletons;

		// Joints
		private Joint leftHand;
		private Joint rightHand;

		// ColorCoordinate Position
		private ColorImagePoint leftHandPoint;
		private ColorImagePoint rightHandPoint;

		// Image Position
		private CvPoint leftHandPosition;
		private CvPoint rightHandPosition;

		// Hand Region Rect
		private CvRect leftHandRect;
		private CvRect rightHandRect;


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
				// Set Color Image
				this.colorImageMat = new CvMat( this.sensor.ColorStream.FrameHeight, this.sensor.ColorStream.FrameWidth, MatrixType.U8C4 );
				Cv.SetData( this.colorImageMat, this.colorPixels, Cv.AUTOSTEP );
				this.leftHandImageMat = new CvMat( Configuration.HAND_REGION_HEIGHT, Configuration.HAND_REGION_WIDTH, MatrixType.U8C4 );
				this.rightHandImageMat = new CvMat( Configuration.HAND_REGION_HEIGHT, Configuration.HAND_REGION_WIDTH, MatrixType.U8C4 );
				this.leftHandDepthImageMat = new CvMat( Configuration.HAND_REGION_HEIGHT, Configuration.HAND_REGION_HEIGHT, MatrixType.U8C4 );
				this.rightHandDepthImageMat = new CvMat( Configuration.HAND_REGION_HEIGHT, Configuration.HAND_REGION_HEIGHT, MatrixType.U8C4 );
				// Add Color Stream Event Handler
				this.sensor.ColorFrameReady += OnColorFrameReady;

				// Enable Depth Stream
				this.sensor.DepthStream.Enable( DepthImageFormat.Resolution640x480Fps30 );
				// Allocate Pixel Data Array
				this.depthPixels = new DepthImagePixel[this.sensor.DepthStream.FramePixelDataLength];
				// Allocate Pixel Depth Color Data Array
				this.depthColorPixels = new byte[this.sensor.DepthStream.FramePixelDataLength * sizeof( int )];
				// Set Depth Image
				this.depthImageMat = new CvMat( this.sensor.DepthStream.FrameHeight, this.sensor.DepthStream.FrameWidth, MatrixType.U8C4 );
				Cv.SetData( this.depthImageMat, this.depthColorPixels, Cv.AUTOSTEP );
				// Add Depth Stream Event Handler
				this.sensor.DepthFrameReady += OnDepthFrameReady;

				// Set Tracking Mode
				this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
				// Enable Skeleton Stream
				this.sensor.SkeletonStream.Enable( new TransformSmoothParameters() { Correction = 0.5f, JitterRadius = 0.05f, MaxDeviationRadius = 0.05f, Prediction = 0.5f, Smoothing = 0.5f } );
				// Allocate Skeleton Data Array
				this.skeletons = new Skeleton[this.sensor.SkeletonStream.FrameSkeletonArrayLength];
				// Add Skeleton Stream Event Handler
				this.sensor.SkeletonFrameReady += OnSkeletonFrameReady;



				// Try To Start Device
				try
				{
					this.sensor.Start();
					Logging.PrintLog( "InitializeKinectSensor", "Kinect is Successfully Initialized" );
				}
				catch (System.Exception ex)
				{
					// Set Null
					this.sensor = null;
					// Set Return Value
					retVal = KinectStatus.Error;
					// Print Error Message
					Logging.PrintErrorLog( "InitializeKinectSensor", "Failed To Start Kinect Device!! - " + ex.ToString() );
				}
			}

			// Return
			return retVal;
		}

		public void DestroyKinect()
		{
			// Check Sensor State
			if ( this.sensor != null )
			{
				this.sensor.Stop();
			}

			// Destroy All Windows
			Cv.DestroyAllWindows();
		}

		private void OnColorFrameReady( object sender, ColorImageFrameReadyEventArgs e )
		{
			// Grab Color Frame
			using ( ColorImageFrame colorFrame = e.OpenColorImageFrame() )
			{

				if ( colorFrame != null )
				{
					// Copy Pixel Data
					colorFrame.CopyPixelDataTo( this.colorPixels );

					// Smooth Image
					//Cv.Smooth( this.colorImageMat, this.colorImageMat, SmoothType.Gaussian );

					// Draw Skeleton Position
					if ( skeletons != null )
					{
						// Find Appropriate Skeleton
						Skeleton targetSkeleton = null;
						for ( int i = 0 ; i < skeletons.Length ; i++ )
						{
							// Skip Invalid State
							if ( skeletons[i] == null )
								continue;

							// Only Fully Tracked Skeleton
							if ( skeletons[i].TrackingState == SkeletonTrackingState.Tracked )
							{
								// Set Target Skeleton - If exists Set to nearest.
								if ( targetSkeleton == null )
									targetSkeleton = skeletons[i];
								else if ( targetSkeleton.Position.Z > skeletons[i].Position.Z )
									targetSkeleton = skeletons[i];
							}
						}

						if ( targetSkeleton != null )
						{
							// Left Hand Position
							this.leftHand = targetSkeleton.Joints[JointType.HandLeft];
							// Check Tracked Status
							if ( this.leftHand.TrackingState == JointTrackingState.Tracked )
							{
								this.leftHandFound = true;

								leftHandPoint = this.sensor.CoordinateMapper.MapSkeletonPointToColorPoint( leftHand.Position, ColorImageFormat.RgbResolution640x480Fps30 );
								this.leftHandPosition.X = leftHandPoint.X;
								this.leftHandPosition.Y = leftHandPoint.Y;
								int topleft_x = leftHandPoint.X - Configuration.HAND_REGION_WIDTH / 2;
								int topleft_y = leftHandPoint.Y - Configuration.HAND_REGION_HEIGHT / 2;

								if ( topleft_x < 0 )
									topleft_x = 0;
								if ( topleft_x + Configuration.HAND_REGION_WIDTH >= this.sensor.ColorStream.FrameWidth )
									topleft_x = this.sensor.ColorStream.FrameWidth - Configuration.HAND_REGION_WIDTH;
								if ( topleft_y < 0 )
									topleft_y = 0;
								if ( topleft_y + Configuration.HAND_REGION_HEIGHT >= this.sensor.ColorStream.FrameHeight )
									topleft_y = this.sensor.ColorStream.FrameHeight - Configuration.HAND_REGION_HEIGHT;

								leftHandRect = new CvRect( topleft_x, topleft_y, Configuration.HAND_REGION_WIDTH, Configuration.HAND_REGION_HEIGHT );
								Cv.Rectangle( this.colorImageMat, leftHandRect, new CvScalar( 0, 0, 255 ), 5 );
								Cv.GetSubRect( this.colorImageMat, out this.leftHandImageMat, leftHandRect );
							}
							else
							{
								this.leftHandFound = false;
							}

							// Right Hand Position
							this.rightHand = targetSkeleton.Joints[JointType.HandRight];
							if ( this.rightHand.TrackingState == JointTrackingState.Tracked )
							{
								this.rightHandFound = true;

								rightHandPoint = this.sensor.CoordinateMapper.MapSkeletonPointToColorPoint( rightHand.Position, ColorImageFormat.RgbResolution640x480Fps30 );
								this.rightHandPosition.X = rightHandPoint.X;
								this.rightHandPosition.Y = rightHandPoint.Y;

								int topleft_x = rightHandPoint.X - Configuration.HAND_REGION_WIDTH / 2;
								int topleft_y = rightHandPoint.Y - Configuration.HAND_REGION_HEIGHT / 2;

								if ( topleft_x < 0 )
									topleft_x = 0;
								if ( topleft_x + Configuration.HAND_REGION_WIDTH >= this.sensor.ColorStream.FrameWidth )
									topleft_x = this.sensor.ColorStream.FrameWidth - Configuration.HAND_REGION_WIDTH;
								if ( topleft_y < 0 )
									topleft_y = 0;
								if ( topleft_y + Configuration.HAND_REGION_HEIGHT >= this.sensor.ColorStream.FrameHeight )
									topleft_y = this.sensor.ColorStream.FrameHeight - Configuration.HAND_REGION_HEIGHT;

								rightHandRect = new CvRect( topleft_x, topleft_y, Configuration.HAND_REGION_WIDTH, Configuration.HAND_REGION_HEIGHT );
								Cv.Rectangle( this.colorImageMat, rightHandRect, new CvScalar( 0, 0, 255 ), 5 );
								Cv.GetSubRect( this.colorImageMat, out this.rightHandImageMat, rightHandRect );
							}
							else
							{
								this.rightHandFound = false;
							}
						}
					}

					// Show Color Image
					Cv.ShowImage( "Color Image", this.colorImageMat );
					//if ( this.leftHandImageMat != null )
					Cv.ShowImage( "Left Hand Region Image", this.leftHandImageMat );
					//if ( this.rightHandImageMat != null )
					Cv.ShowImage( "RIght Hand Region Image", this.rightHandImageMat );
				}
			}
		}

		private void OnDepthFrameReady( object sender, DepthImageFrameReadyEventArgs e )
		{
			// Grab Depth Frame
			using ( DepthImageFrame depthFrame = e.OpenDepthImageFrame() )
			{
				if ( depthFrame != null )
				{
					// Copy Depth Pixel Data
					depthFrame.CopyDepthImagePixelDataTo( this.depthPixels );

					// Get Reliable Min Max Value
					int minDepth = depthFrame.MinDepth;
					int maxDepth = depthFrame.MaxDepth;

					// Convert Depth to RGB
					int colorPixelIndex = 0;
					for ( int i = 0 ; i < this.depthPixels.Length ; i++ )
					{
						// Get Depth Value For This Pixel
						short depthValue = depthPixels[i].Depth;

						// Convert Intensity To Byte
						// TODO Make A lookup Table for Performance
						depthValue = (short)( depthValue >= minDepth && depthValue <= maxDepth ? depthValue : 0 );
						// Normalize
						byte intensity = (byte)( depthValue * 255 / ( maxDepth - minDepth ) );

						// BGR
						this.depthColorPixels[colorPixelIndex++] = intensity;
						this.depthColorPixels[colorPixelIndex++] = intensity;
						this.depthColorPixels[colorPixelIndex++] = intensity;
						// Skip A
						colorPixelIndex++;
					}

					if ( this.leftHand.TrackingState == JointTrackingState.Tracked )
					{
						DepthImagePoint ldHand = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint( this.leftHand.Position, DepthImageFormat.Resolution640x480Fps30 );

						int topleft_x = ldHand.X - Configuration.HAND_REGION_WIDTH / 2;
						int topleft_y = ldHand.Y - Configuration.HAND_REGION_HEIGHT / 2;

						if ( topleft_x < 0 )
							topleft_x = 0;
						if ( topleft_x + Configuration.HAND_REGION_WIDTH >= this.sensor.ColorStream.FrameWidth )
							topleft_x = this.sensor.ColorStream.FrameWidth - Configuration.HAND_REGION_WIDTH;
						if ( topleft_y < 0 )
							topleft_y = 0;
						if ( topleft_y + Configuration.HAND_REGION_HEIGHT >= this.sensor.ColorStream.FrameHeight )
							topleft_y = this.sensor.ColorStream.FrameHeight - Configuration.HAND_REGION_HEIGHT;

						CvRect ldHandRect = new CvRect( topleft_x, topleft_y, Configuration.HAND_REGION_WIDTH, Configuration.HAND_REGION_HEIGHT );
						Cv.Rectangle( depthImageMat, ldHandRect, new CvScalar( 0, 0, 255 ), 5 );
						Cv.GetSubRect( this.depthImageMat, out this.leftHandDepthImageMat, ldHandRect );

					}
					if ( this.rightHand.TrackingState == JointTrackingState.Tracked )
					{
						DepthImagePoint rdHand = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint( this.rightHand.Position, DepthImageFormat.Resolution640x480Fps30 );

						int topleft_x = rdHand.X - Configuration.HAND_REGION_WIDTH / 2;
						int topleft_y = rdHand.Y - Configuration.HAND_REGION_HEIGHT / 2;

						if ( topleft_x < 0 )
							topleft_x = 0;
						if ( topleft_x + Configuration.HAND_REGION_WIDTH >= this.sensor.ColorStream.FrameWidth )
							topleft_x = this.sensor.ColorStream.FrameWidth - Configuration.HAND_REGION_WIDTH;
						if ( topleft_y < 0 )
							topleft_y = 0;
						if ( topleft_y + Configuration.HAND_REGION_HEIGHT >= this.sensor.ColorStream.FrameHeight )
							topleft_y = this.sensor.ColorStream.FrameHeight - Configuration.HAND_REGION_HEIGHT;

						CvRect rdHandRect = new CvRect( topleft_x, topleft_y, Configuration.HAND_REGION_WIDTH, Configuration.HAND_REGION_HEIGHT );
						Cv.Rectangle( depthImageMat, rdHandRect, new CvScalar( 0, 0, 255 ), 5 );
						Cv.GetSubRect( this.depthImageMat, out this.rightHandDepthImageMat, rdHandRect );

					}

					// Show Depth Image
					Cv.ShowImage( "Depth Image", this.depthImageMat );
					Cv.ShowImage( "Left Hand Region Depth Image", this.leftHandDepthImageMat );
					Cv.ShowImage( "Right Hand Region Depth Image", this.rightHandDepthImageMat );
				}
			}
		}

		private void OnSkeletonFrameReady( object sender, SkeletonFrameReadyEventArgs e )
		{
			// Grab Skeleton Frame
			using ( SkeletonFrame skeletonFrame = e.OpenSkeletonFrame() )
			{
				if ( skeletonFrame != null && this.skeletons != null )
				{
					// Copy Skeleton Datas
					skeletonFrame.CopySkeletonDataTo( skeletons );
				}
			}

		}
	}
}
