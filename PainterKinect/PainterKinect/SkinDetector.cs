using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using OpenCvSharp;
using System.Runtime.InteropServices;

namespace PainterKinect
{
	class SkinDetector
	{
		private const string SKIN_MODEL_DATA_FILENAME = "StatisticalSkinColorModelFloat.data";
		private float[] colorModelData;
		private bool isInitialized = false;

		public SkinDetector()
		{
			try
			{
				// Set Reader
				using ( BinaryReader reader = new BinaryReader( File.Open( SKIN_MODEL_DATA_FILENAME, FileMode.Open ) ) )
				{
					// Allocate Memory
					this.colorModelData = new float[256 * 256 * 256];
					for ( int i = 0 ; i < this.colorModelData.Length ; i++ )
						this.colorModelData[i] = reader.ReadSingle();

					// Initialized
					this.isInitialized = true;
				}
			}
			catch (System.Exception ex)
			{
				// Error Loading Skin Color Model
				Logging.PrintErrorLog( "SkinColorModel", "Loading Skin Color Model Data Failed!! Check Input File." + ex.ToString() );
			}
		}

		public void FilterSkinColorRegion( IplImage rgbImage_src, IplImage rgbImage_dst, float colorThreshold = 0.4f )
		{
			if ( !isInitialized )
				return;

			// Color Model Value
			float nVal;

			// Get Image Channel
			int channel = rgbImage_src.NChannels;
			int widthStep = rgbImage_src.WidthStep;
			int channel_dst = rgbImage_dst.NChannels;
			int widthStep_dst = rgbImage_dst.WidthStep;

			// Image Data Ptr
			int b, g, r;

			for ( int dy = 0 ; dy < rgbImage_src.Height ; dy++ )
			{
				for ( int dx = 0 ; dx < rgbImage_src.Width ; dx++ )
				{
					// Get BGR Value
					unsafe
					{
						b = (int)rgbImage_src.ImageDataPtr[dy * widthStep + channel * dx + 0]; // B
						g = (int)rgbImage_src.ImageDataPtr[dy * widthStep + channel * dx + 1]; // G
						r = (int)rgbImage_src.ImageDataPtr[dy * widthStep + channel * dx + 2]; // R
					}

					nVal = this.colorModelData[b + 256 * g + 256 * 256 * r];

					// Determine & Filter Skin Region
					if ( nVal >= colorThreshold )
					{
						// Skin Color
						unsafe
						{
							rgbImage_dst.ImageDataPtr[dy * widthStep_dst + channel_dst * dx] = (byte)255;
						}
					}
					else
					{
						// Not Skin Color
						unsafe
						{
							rgbImage_dst.ImageDataPtr[dy * widthStep_dst + channel_dst * dx] = (byte)0;
						}
					}
				}
			}

		}



		public void FilterCenteredRegion( IplImage handImage )
		{
			// 
			int centerX = handImage.Width / 2;
			int centerY = handImage.Height / 2;

			// Labeling
			for ( int dy = 0 ; dy < handImage.Height ; dy++ )
			{
				for ( int dx = 0 ; dx < handImage.Width ; dx++ )
				{
					
				}
			}
		}
	}
}
