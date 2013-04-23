using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using OpenCvSharp;
using System.Runtime.InteropServices;

namespace PainterKinect
{
	class SkinColorModel
	{
		private const string SKIN_MODEL_DATA_FILENAME = "StatisticalSkinColorModelFloat.data";
		private float[] colorModelData;
		private bool isInitialized = false;

		public SkinColorModel()
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

		public int DetectSkinRegion( IplImage rgbImage, float colorThreshold = 0.4f, int areaThreshold = 1000 )
		{
			if ( !isInitialized )
				return -1;

			int channel = rgbImage.NChannels;

			// Area Count
			int areaCnt = 0;

			// Image Data Ptr
			byte b, g, r;

			for ( int dy = 0 ; dy < rgbImage.Height ; dy++ )
			{
				for ( int dx = 0 ; dx < rgbImage.Width ; dx++ )
				{
					unsafe
					{
						b = rgbImage.ImageDataPtr[dy * rgbImage.WidthStep + channel * dx + 0];
						g = rgbImage.ImageDataPtr[dy * rgbImage.WidthStep + channel * dx + 1];
						r = rgbImage.ImageDataPtr[dy * rgbImage.WidthStep + channel * dx + 2];
					}

					float nVal = this.colorModelData[(int)b + 256 * (int)g + 256 * 256 * (int)r ];

					if ( nVal >= colorThreshold )
					{
						areaCnt++;
					}
					else
					{
						unsafe
						{
							// B G R
							rgbImage.ImageDataPtr[dy * rgbImage.WidthStep + channel * dx + 0] = (byte)0;
							rgbImage.ImageDataPtr[dy * rgbImage.WidthStep + channel * dx + 1] = (byte)0;
							rgbImage.ImageDataPtr[dy * rgbImage.WidthStep + channel * dx + 2] = (byte)0;
						}
					}
				}
			}


			if ( areaCnt > areaThreshold )
				return 1;
			else
				return 0;
		}
	}
}
