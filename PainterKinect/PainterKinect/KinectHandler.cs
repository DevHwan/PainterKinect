using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Kinect;
using System.Windows.Media.Imaging;
using System.Windows.Media;
using System.Windows;
using OpenCvSharp;
using OpenCvSharp.CPlusPlus;


//////////////////////////////////////////////////////////////////////////
// Depth Value Range
//  - byte : 0 ~ 255 (1 is far, 255 is near, 0 is unstable value)
// Color Value Range
//  - color : 0 ~ 255 * 3 (B G R)
//////////////////////////////////////////////////////////////////////////

namespace PainterKinect
{
	public class KinectHandler
	{
		// Configuration Constants
		private const int MAX_DEPTH_RANGE = 2000;


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
		private IplImage leftHandSkinImage;
		private IplImage rightHandSkinImage;

		// Hand Region Depth Image
		private IplImage leftHandDepthImage;
		private IplImage rightHandDepthImage;

		// Object Remained Image
		private IplImage leftObjectRemainedImage;
		private IplImage rightObjectRemainedImage;

		// Object Only Image
		private IplImage leftObjectImage;
		private IplImage rightObjectImage;

		// Objects Color Image
		private IplImage leftObjectR;
		private IplImage leftObjectG;
		private IplImage leftObjectB;
		private IplImage rightObjectR;
		private IplImage rightObjectG;
		private IplImage rightObjectB;
		private IplImage tempObjectA;

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
		private SkinDetector skinDetector;

		// Min Max Depth Value
		private int maxDepth = 0;
		private int minDepth = int.MaxValue;

		// Depth Value Lookup Table
		private byte[] depthLookupTable;

		// Exit Status
		private bool isShutDown = false;

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
				this.maxDepth = MAX_DEPTH_RANGE;// this.sensor.DepthStream.MaxDepth;

				// Allocate Depth Lookup Table
				this.depthLookupTable = new byte[this.maxDepth];
				for ( int i = 0 ; i < maxDepth ; i++ )
				{
					short depthValue = (short)i;
					depthValue = (short)( depthValue > this.minDepth ? this.maxDepth - depthValue : 0 );
					this.depthLookupTable[i] = (byte)( ( depthValue * 255 / ( this.maxDepth - this.minDepth ) ) );
				}

				// Allocate Color Image
				this.colorImage = Cv.CreateImage( new CvSize( this.sensor.ColorStream.FrameWidth, this.sensor.ColorStream.FrameHeight ), BitDepth.U8, 4 );
				// Allocate Depth Image
				this.depthImage = Cv.CreateImage( new CvSize( this.sensor.DepthStream.FrameWidth, this.sensor.DepthStream.FrameHeight ), BitDepth.U8, 1 );

				// Allocate Pixel RAW Data Array
				this.depthPixels = new DepthImagePixel[this.sensor.DepthStream.FramePixelDataLength];

				// Allocate Hand Image
				this.leftHandImage = Cv.CreateImage( new CvSize( Configuration.HAND_REGION_WIDTH, Configuration.HAND_REGION_HEIGHT ), BitDepth.U8, 4 );
				this.rightHandImage = Cv.CreateImage( new CvSize( Configuration.HAND_REGION_WIDTH, Configuration.HAND_REGION_HEIGHT ), BitDepth.U8, 4 );
				this.leftHandSkinImage = Cv.CreateImage( new CvSize( Configuration.HAND_REGION_WIDTH, Configuration.HAND_REGION_HEIGHT ), BitDepth.U8, 1 );
				this.rightHandSkinImage = Cv.CreateImage( new CvSize( Configuration.HAND_REGION_WIDTH, Configuration.HAND_REGION_HEIGHT ), BitDepth.U8, 1 );

				// Allocate Hand Depth Image
				this.leftHandDepthImage = Cv.CreateImage( new CvSize( Configuration.HAND_REGION_WIDTH, Configuration.HAND_REGION_HEIGHT ), BitDepth.U8, 1 );
				this.rightHandDepthImage = Cv.CreateImage( new CvSize( Configuration.HAND_REGION_WIDTH, Configuration.HAND_REGION_HEIGHT ), BitDepth.U8, 1 );

				// Allocate Object Image
				this.leftObjectRemainedImage = Cv.CreateImage( new CvSize( Configuration.HAND_REGION_WIDTH, Configuration.HAND_REGION_HEIGHT ), BitDepth.U8, 1 );
				this.rightObjectRemainedImage = Cv.CreateImage( new CvSize( Configuration.HAND_REGION_WIDTH, Configuration.HAND_REGION_HEIGHT ), BitDepth.U8, 1 );

				// Allocate Object Only Image
				this.leftObjectImage = Cv.CreateImage( new CvSize( Configuration.OBJECT_REGION_WIDTH, Configuration.OBJECT_REGION_HEIGHT ), BitDepth.U8, 4 );
				this.rightObjectImage = Cv.CreateImage( new CvSize( Configuration.OBJECT_REGION_WIDTH, Configuration.OBJECT_REGION_HEIGHT ), BitDepth.U8, 4 );

				// RGB Image Datas
				this.leftObjectR = Cv.CreateImage( new CvSize( Configuration.HAND_REGION_WIDTH, Configuration.HAND_REGION_HEIGHT ), BitDepth.U8, 1 );
				this.leftObjectG = Cv.CreateImage( new CvSize( Configuration.HAND_REGION_WIDTH, Configuration.HAND_REGION_HEIGHT ), BitDepth.U8, 1 );
				this.leftObjectB = Cv.CreateImage( new CvSize( Configuration.HAND_REGION_WIDTH, Configuration.HAND_REGION_HEIGHT ), BitDepth.U8, 1 );
				this.rightObjectR = Cv.CreateImage( new CvSize( Configuration.HAND_REGION_WIDTH, Configuration.HAND_REGION_HEIGHT ), BitDepth.U8, 1 );
				this.rightObjectG = Cv.CreateImage( new CvSize( Configuration.HAND_REGION_WIDTH, Configuration.HAND_REGION_HEIGHT ), BitDepth.U8, 1 );
				this.rightObjectB = Cv.CreateImage( new CvSize( Configuration.HAND_REGION_WIDTH, Configuration.HAND_REGION_HEIGHT ), BitDepth.U8, 1 );
				this.tempObjectA = Cv.CreateImage( new CvSize( Configuration.HAND_REGION_WIDTH, Configuration.HAND_REGION_HEIGHT ), BitDepth.U8, 1 );

				// Set Tracking Mode
				this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
				// Enable Skeleton Stream
				this.sensor.SkeletonStream.Enable( new TransformSmoothParameters() { Correction = 0.5f, JitterRadius = 0.05f, MaxDeviationRadius = 0.04f, Prediction = 0.5f, Smoothing = 0.5f } );
				// Allocate Skeleton Data Array
				this.skeletons = new Skeleton[this.sensor.SkeletonStream.FrameSkeletonArrayLength];

				// Initialize SkinColorModel Module
				this.skinDetector = new SkinDetector();

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
			// Set ShutDown Mode
			this.isShutDown = true;

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

			// Hand Image
			Cv.ReleaseImage( this.leftHandImage );
			Cv.ReleaseImage( this.rightHandImage );
			// Skin Image
			Cv.ReleaseImage( this.leftHandSkinImage );
			Cv.ReleaseImage( this.rightHandSkinImage );
			// Hand Depth Image
			Cv.ReleaseImage( this.leftHandDepthImage );
			Cv.ReleaseImage( this.rightHandDepthImage );
			// Object Image
			Cv.ReleaseImage( this.leftObjectRemainedImage );
			Cv.ReleaseImage( this.rightObjectRemainedImage );
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
					// Get Current Pixel's Depth Value
					unsafe
					{
						currentNearVal = depthImage.ImageDataPtr[dy * depthImage.WidthStep + depthImage.NChannels * dx];
					}

					// Remove Far Objects
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

		private void CropNear( IplImage inputDepthImage )
		{
			int channel = inputDepthImage.NChannels;
			int val;
			int cnt = 0;
			int max = 0;
			int min = int.MaxValue;

			// Find Mean Value
			for ( int dy = 0 ; dy < inputDepthImage.Height ; dy++ )
			{
				for ( int dx = 0 ; dx < inputDepthImage.Width ; dx++ )
				{
					unsafe
					{
						val = inputDepthImage.ImageDataPtr[dy * inputDepthImage.WidthStep + channel * dx];
						if ( val != 0 )
						{
							cnt++;

							if ( max < val )
								max = val;
							if ( min > val )
								min = val;
						}

					}
				}
			}

			val = ( min + max ) / 2;

			Cv.Threshold( inputDepthImage, inputDepthImage, val, 255.0, ThresholdType.ToZero );
		}

		private void OnAllFrameReady( object sender, AllFramesReadyEventArgs e )
		{
			if ( this.isShutDown )
				return;

			#region Skeleton Data Process Region
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
			#endregion

			#region Depth Data Process Region
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
					for ( int i = 0 ; i < this.depthPixels.Length ; i++ )
					{
						// Get Depth Value For This Pixel
						depthValue = depthPixels[i].Depth;

						unsafe
						{
							if ( depthValue >= this.maxDepth || depthValue <= this.minDepth )
							{
								this.depthImage.ImageDataPtr[i] = (byte)0;
							}
							else
							{
								this.depthImage.ImageDataPtr[i] = this.depthLookupTable[depthValue];
							}
						}
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
						//Cv.Rectangle( this.depthImage, ldHandRect, new CvScalar( 0, 0, 255 ), 5 ); // Used for Visualization
						Cv.SetImageROI( this.depthImage, ldHandRect );
						Cv.Copy( this.depthImage, this.leftHandDepthImage );
						Cv.ResetImageROI( this.depthImage );

						// Filter Hand Depth Image
						Cv.Smooth( this.leftHandDepthImage, this.leftHandDepthImage, SmoothType.Median );

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
						//Cv.Rectangle( this.depthImage, rdHandRect, new CvScalar( 0, 0, 255 ), 5 ); // Used for Visualization
						Cv.SetImageROI( this.depthImage, rdHandRect );
						Cv.Copy( this.depthImage, this.rightHandDepthImage );
						Cv.ResetImageROI( this.depthImage );

						// Filter Hand Depth Image
						Cv.Smooth( this.rightHandDepthImage, this.rightHandDepthImage, SmoothType.Median );
					}
				}
			}
			#endregion

			#region Color Data Process Region
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
								Cv.Rectangle( this.colorImage, this.leftHandRect, new CvScalar( 0, 0, 255 ), 1 ); // Used for Visualization
								Cv.SetImageROI( this.colorImage, this.leftHandRect );
								// Copy Data
								Cv.Copy( this.colorImage, this.leftHandImage );
								// Reset ROI
								Cv.ResetImageROI( this.colorImage );

								// Smooth Color Hand Image
								Cv.Smooth( this.leftHandImage, this.leftHandImage, SmoothType.Median );

								// Only Hand Region
								CropNear( this.leftHandDepthImage );

								// Filter With Depth Image
								FilterFarObjects( this.leftHandImage, this.leftHandDepthImage );

								// Detect By Skin Color Model
								this.skinDetector.FilterSkinColorRegion( this.leftHandImage, this.leftHandSkinImage, 0.25f );
								// Smooth Color Hand Skin Image
								//Cv.Smooth( this.leftHandSkinImage, this.leftHandSkinImage, SmoothType.Median );
								Cv.Smooth( this.leftHandSkinImage, this.leftHandSkinImage, SmoothType.Median, 5 );
								Cv.Erode( this.leftHandSkinImage, this.leftHandSkinImage );
								Cv.Dilate( this.leftHandSkinImage, this.leftHandSkinImage );
								

								// Find Object
								Cv.Sub( this.leftHandDepthImage, this.leftHandSkinImage, this.leftObjectRemainedImage );
								Cv.Erode( this.leftObjectRemainedImage, this.leftObjectRemainedImage );
								Cv.Smooth( this.leftObjectRemainedImage, this.leftObjectRemainedImage, SmoothType.Median );

								// Filter Objects Only
								FilterFarObjects( this.leftHandImage, this.leftObjectRemainedImage );

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
								Cv.Rectangle( this.colorImage, this.rightHandRect, new CvScalar( 0, 0, 255 ), 1 ); // Used for Visualization
								Cv.SetImageROI( this.colorImage, this.rightHandRect );
								// Copy Data
								Cv.Copy( this.colorImage, this.rightHandImage );
								// Reset ROI
								Cv.ResetImageROI( this.colorImage );

								// Smooth Color Hand Image
								Cv.Smooth( this.rightHandImage, this.rightHandImage, SmoothType.Median );

								CropNear( this.rightHandDepthImage );

								// Filter With Depth Image
								FilterFarObjects( this.rightHandImage, this.rightHandDepthImage );

								// Detect By Skin Color Model
								this.skinDetector.FilterSkinColorRegion( this.rightHandImage, this.rightHandSkinImage, 0.25f );
								// Smooth Color Hand Skin Image
								//Cv.Smooth( this.rightHandSkinImage, this.rightHandSkinImage, SmoothType.Median );
								Cv.Smooth( this.rightHandSkinImage, this.rightHandSkinImage, SmoothType.Median, 5 );
								Cv.Erode( this.rightHandSkinImage, this.rightHandSkinImage );
								Cv.Dilate( this.rightHandSkinImage, this.rightHandSkinImage );

								// Find Object
								Cv.Sub( this.rightHandDepthImage, this.rightHandSkinImage, this.rightObjectRemainedImage );
								Cv.Erode( this.rightObjectRemainedImage, this.rightObjectRemainedImage );
								Cv.Smooth( this.rightObjectRemainedImage, this.rightObjectRemainedImage, SmoothType.Median );

								// Filter Objects Only
								FilterFarObjects( this.rightHandImage, this.rightObjectRemainedImage );

							}
							else
							{
								this.rightHandFound = false;
							}
						}
					}
			#endregion

					#region Image Display Region
					// Show Depth Image
					Cv.ShowImage( "Depth Image", this.depthImage );
					Cv.ShowImage( "Left Hand Depth Image", this.leftHandDepthImage );
					Cv.ShowImage( "Right Hand Depth Image", this.rightHandDepthImage );

					// Show Color Image
					Cv.ShowImage( "Color Image", this.colorImage );
					Cv.ShowImage( "Left Hand Image", this.leftHandImage );
					Cv.ShowImage( "Right Hand Image", this.rightHandImage );
					Cv.ShowImage( "Left Hand Skin Image", this.leftHandSkinImage );
					Cv.ShowImage( "Right Hand Skin Image", this.rightHandSkinImage );

					// Show Object Only Image
					Cv.ShowImage( "Left Hand Object Image", this.leftObjectRemainedImage );
					Cv.ShowImage( "Right Hand Object Image", this.rightObjectRemainedImage );


					Cv.ShowImage( "Left B", this.leftObjectB );
					Cv.ShowImage( "Left G", this.leftObjectG );
					Cv.ShowImage( "Left R", this.leftObjectR );
					Cv.ShowImage( "Right B", this.rightObjectB );
					Cv.ShowImage( "Right G", this.rightObjectG );
					Cv.ShowImage( "Right R", this.rightObjectR );
					#endregion
				}
			}
		}
	}
}
