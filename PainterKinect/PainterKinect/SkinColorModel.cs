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

		public void FilterSkinRegion( IplImage rgbImage, float colorThreshold = 0.4f )
		{
			if ( !isInitialized )
				return;

			// Color Model Value
			float nVal;

			// Get Image Channel
			int channel = rgbImage.NChannels;


			// Image Data Ptr
			int b, g, r;

			for ( int dy = 0 ; dy < rgbImage.Height ; dy++ )
			{
				for ( int dx = 0 ; dx < rgbImage.Width ; dx++ )
				{
					// Get BGR Value
					unsafe
					{
						b = (int)rgbImage.ImageDataPtr[dy * rgbImage.WidthStep + channel * dx + 0]; // B
						g = (int)rgbImage.ImageDataPtr[dy * rgbImage.WidthStep + channel * dx + 1]; // G
						r = (int)rgbImage.ImageDataPtr[dy * rgbImage.WidthStep + channel * dx + 2]; // R
					}

					nVal = this.colorModelData[b + 256 * g + 256 * 256 * r];

					// Determine & Filter Skin Region
					if ( nVal < colorThreshold )
					{
						// Not Skin Color
						unsafe
						{
							rgbImage.ImageDataPtr[dy * rgbImage.WidthStep + channel * dx + 0] = (byte)0; // B
							rgbImage.ImageDataPtr[dy * rgbImage.WidthStep + channel * dx + 1] = (byte)0; // G
							rgbImage.ImageDataPtr[dy * rgbImage.WidthStep + channel * dx + 2] = (byte)0; // R
						}
					}
				}
			}
		}
	}
}
