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
		private DepthImagePixel[] depthPixels;

		// Color & Depth Image
		private IplImage colorImage;
		private IplImage depthImage;

		// Hand Region Image
		private IplImage leftHandImage;
		private IplImage rightHandImage;

		// Hand Region Depth Image
		private IplImage leftHandDepthImage;
		private IplImage rightHandDepthImage;

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

		// Skin Color Model Module
		private SkinColorModel skinModel;

		// Min Max Depth Value
		private int maxDepth = 0;
		private int minDepth = int.MaxValue;

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
				// Enable Depth Stream
				this.sensor.DepthStream.Enable( DepthImageFormat.Resolution640x480Fps30 );

				// Set Min Max Depth Value
				this.minDepth = this.sensor.DepthStream.MinDepth;
				this.maxDepth = this.sensor.DepthStream.MaxDepth;

				// Allocate Color Image
				this.colorImage = Cv.CreateImage( new CvSize( this.sensor.ColorStream.FrameWidth, this.sensor.ColorStream.FrameHeight ), BitDepth.U8, 4 );
				// Allocate Depth Image
				this.depthImage = Cv.CreateImage( new CvSize( this.sensor.DepthStream.FrameWidth, this.sensor.DepthStream.FrameHeight ), BitDepth.U8, 1 );

				// Allocate Pixel RAW Data Array
				this.depthPixels = new DepthImagePixel[this.sensor.DepthStream.FramePixelDataLength];

				// Allocate Hand Image
				this.leftHandImage = Cv.CreateImage( new CvSize( Configuration.HAND_REGION_WIDTH, Configuration.HAND_REGION_HEIGHT ), BitDepth.U8, 4 );
				this.rightHandImage = Cv.CreateImage( new CvSize( Configuration.HAND_REGION_WIDTH, Configuration.HAND_REGION_HEIGHT ), BitDepth.U8, 4 );

				// Allocate Hand Depth Image
				this.leftHandDepthImage = Cv.CreateImage( new CvSize( Configuration.HAND_REGION_WIDTH, Configuration.HAND_REGION_HEIGHT ), BitDepth.U8, 1 );
				this.rightHandDepthImage = Cv.CreateImage( new CvSize( Configuration.HAND_REGION_WIDTH, Configuration.HAND_REGION_HEIGHT ), BitDepth.U8, 1 );

				// Set Tracking Mode
				this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
				// Enable Skeleton Stream
				this.sensor.SkeletonStream.Enable( new TransformSmoothParameters() { Correction = 0.5f, JitterRadius = 0.05f, MaxDeviationRadius = 0.05f, Prediction = 0.5f, Smoothing = 0.5f } );
				// Allocate Skeleton Data Array
				this.skeletons = new Skeleton[this.sensor.SkeletonStream.FrameSkeletonArrayLength];

				// Initialize SkinColorModel Module
				this.skinModel = new SkinColorModel();

				// Frame Handler
				this.sensor.AllFramesReady += OnAllFrameReady;

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

			// Release IplImage Memory
			Cv.ReleaseImage( this.colorImage );
			Cv.ReleaseImage( this.depthImage );
			Cv.ReleaseImage( this.leftHandImage );
			Cv.ReleaseImage( this.leftHandDepthImage );
			Cv.ReleaseImage( this.rightHandImage );
			Cv.ReleaseImage( this.rightHandDepthImage );
			// 
		}

		private void FilterFarObjects( IplImage inputImage, IplImage depthImage, byte distance = 50 )
		{
			// Check!! Input & Depth must have same size

			int channel = inputImage.NChannels;

			byte currentNearVal;

			// Remove Too Far Objects
			for ( int dy = 0 ; dy < inputImage.Height ; dy++ )
			{
				for ( int dx = 0 ; dx < inputImage.Width ; dx++ )
				{
					unsafe
					{
						currentNearVal = depthImage.ImageDataPtr[dy * depthImage.WidthStep + depthImage.NChannels * dx + 0];
					}

					if ( currentNearVal < distance )
					{
						unsafe
						{
							inputImage.ImageDataPtr[dy * inputImage.WidthStep + channel * dx + 0] = (byte)0;
							inputImage.ImageDataPtr[dy * inputImage.WidthStep + channel * dx + 1] = (byte)0;
							inputImage.ImageDataPtr[dy * inputImage.WidthStep + channel * dx + 2] = (byte)0;
						}
					}
				}
			}
		}

		private void OnAllFrameReady( object sender, AllFramesReadyEventArgs e )
		{
			// 1. Process Skeleton Data
			// Grab Skeleton Frame
			using ( SkeletonFrame skeletonFrame = e.OpenSkeletonFrame() )
			{
				if ( skeletonFrame != null && this.skeletons != null )
				{
					// Copy Skeleton Datas
					skeletonFrame.CopySkeletonDataTo( skeletons );
				}
			}

			// 2. Process Depth Data
			// Grab Depth Frame
			short depthValue;
			using ( DepthImageFrame depthFrame = e.OpenDepthImageFrame() )
			{
				if ( depthFrame != null )
				{
					// Copy Depth Pixel Data
					depthFrame.CopyDepthImagePixelDataTo( this.depthPixels );

					// Convert Depth to RGB
					int colorPixelIndex = 0;
					for ( int i = 0 ; i < this.depthPixels.Length ; i++ )
					{
						// Get Depth Value For This Pixel
						depthValue = depthPixels[i].Depth;

						// Convert Intensity To Byte
						// TODO Make A lookup Table for Performance
						depthValue = (short)( depthValue >= this.minDepth && depthValue <= this.maxDepth ? ~depthValue : 0 );

						unsafe
						{
							this.depthImage.ImageDataPtr[colorPixelIndex++] = (byte)( depthValue * 255 / ( this.maxDepth - this.minDepth ) );
						}
					}

					// Filter Depth Image
					//Cv.Smooth( this.depthImage, this.depthImage, SmoothType.Median );
					//Cv.Dilate( this.depthImage, this.depthImage );

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
						//Cv.Rectangle( this.depthImage, ldHandRect, new CvScalar( 0, 0, 255 ), 5 );
						Cv.SetImageROI( this.depthImage, ldHandRect );
						Cv.Copy( this.depthImage, this.leftHandDepthImage );
						Cv.ResetImageROI( this.depthImage );

						// Filter Hand Depth Image
						Cv.Smooth( this.leftHandDepthImage, this.leftHandDepthImage, SmoothType.Median );
						//Cv.Erode( this.leftHandDepthImage, this.leftHandDepthImage);
						Cv.Dilate( this.leftHandDepthImage, this.leftHandDepthImage );

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
						//Cv.Rectangle( this.depthImage, rdHandRect, new CvScalar( 0, 0, 255 ), 5 );
						Cv.SetImageROI( this.depthImage, rdHandRect );
						Cv.Copy( this.depthImage, this.rightHandDepthImage );
						Cv.ResetImageROI( this.depthImage );

						// Filter Hand Depth Image
						Cv.Smooth( this.rightHandDepthImage, this.rightHandDepthImage, SmoothType.Median );
						//Cv.Erode( this.rightHandDepthImage, this.rightHandDepthImage );
						Cv.Dilate( this.rightHandDepthImage, this.rightHandDepthImage );
					}

					// Show Depth Image
					Cv.ShowImage( "Depth Image", this.depthImage );
					Cv.ShowImage( "Left Hand Region Depth Image", this.leftHandDepthImage );
					Cv.ShowImage( "Right Hand Region Depth Image", this.rightHandDepthImage );
				}
			}

			// 3. Process Color Data
			// Grab Color Frame
			using ( ColorImageFrame colorFrame = e.OpenColorImageFrame() )
			{

				if ( colorFrame != null )
				{
					// Copy Pixel Data
					unsafe
					{
						colorFrame.CopyPixelDataTo( this.colorImage.ImageData, colorFrame.PixelDataLength );
					}

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

								// Bound Check
								if ( topleft_x < 0 )
									topleft_x = 0;
								if ( topleft_x + Configuration.HAND_REGION_WIDTH >= this.sensor.ColorStream.FrameWidth )
									topleft_x = this.sensor.ColorStream.FrameWidth - Configuration.HAND_REGION_WIDTH;
								if ( topleft_y < 0 )
									topleft_y = 0;
								if ( topleft_y + Configuration.HAND_REGION_HEIGHT >= this.sensor.ColorStream.FrameHeight )
									topleft_y = this.sensor.ColorStream.FrameHeight - Configuration.HAND_REGION_HEIGHT;

								// Set Hand Position
								leftHandRect = new CvRect( topleft_x, topleft_y, Configuration.HAND_REGION_WIDTH, Configuration.HAND_REGION_HEIGHT );
								//Cv.Rectangle( this.colorImage, this.leftHandRect, new CvScalar( 0, 0, 255 ), 5 ); // Used for Visualization
								Cv.SetImageROI( this.colorImage, this.leftHandRect );
								// Copy Data
								Cv.Copy( this.colorImage, this.leftHandImage );
								// Reset ROI
								Cv.ResetImageROI( this.colorImage );

								// Smooth Color Hand Image
								Cv.Smooth( this.leftHandImage, this.leftHandImage, SmoothType.Median );

								// Filter With Depth Image
								FilterFarObjects( this.leftHandImage, this.leftHandDepthImage );

								// Detect By Skin Color Model
								this.skinModel.DetectSkinRegion( this.leftHandImage, 0.4f, 1000 );

								// Smooth Color Hand Image
								Cv.Smooth( this.leftHandImage, this.leftHandImage, SmoothType.Median );
								// Erode & Dilate
								Cv.Erode( this.leftHandImage, this.leftHandImage );
								Cv.Dilate( this.leftHandImage, this.leftHandImage );
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

								// Bound Check
								if ( topleft_x < 0 )
									topleft_x = 0;
								if ( topleft_x + Configuration.HAND_REGION_WIDTH >= this.sensor.ColorStream.FrameWidth )
									topleft_x = this.sensor.ColorStream.FrameWidth - Configuration.HAND_REGION_WIDTH;
								if ( topleft_y < 0 )
									topleft_y = 0;
								if ( topleft_y + Configuration.HAND_REGION_HEIGHT >= this.sensor.ColorStream.FrameHeight )
									topleft_y = this.sensor.ColorStream.FrameHeight - Configuration.HAND_REGION_HEIGHT;

								// Set Hand Position
								rightHandRect = new CvRect( topleft_x, topleft_y, Configuration.HAND_REGION_WIDTH, Configuration.HAND_REGION_HEIGHT );
								//Cv.Rectangle( this.colorImage, this.rightHandRect, new CvScalar( 0, 0, 255 ), 5 ); // Used for Visualization
								Cv.SetImageROI( this.colorImage, this.rightHandRect );
								// Copy Data
								Cv.Copy( this.colorImage, this.rightHandImage );
								// Reset ROI
								Cv.ResetImageROI( this.colorImage );

								// Smooth Color Hand Image
								Cv.Smooth( this.rightHandImage, this.rightHandImage, SmoothType.Median );

								// Filter With Depth Image
								FilterFarObjects( this.rightHandImage, this.rightHandDepthImage );

								// Detect By Skin Color Model
								this.skinModel.DetectSkinRegion( this.rightHandImage, 0.4f, 1000 );

								// Smooth Color Hand Image
								Cv.Smooth( this.rightHandImage, this.rightHandImage, SmoothType.Median );
								// Erode & Dilate
								Cv.Erode( this.rightHandImage, this.rightHandImage );
								Cv.Dilate( this.rightHandImage, this.rightHandImage );
							}
							else
							{
								this.rightHandFound = false;
							}
						}
					}

					// Show Color Image
					Cv.ShowImage( "Color Image", this.colorImage );
					//if ( this.leftHandImageMat != null )
					Cv.ShowImage( "Left Hand Region Image", this.leftHandImage );
					//if ( this.rightHandImageMat != null )
					Cv.ShowImage( "RIght Hand Region Image", this.rightHandImage );

				}
			}
		}
	}
}
