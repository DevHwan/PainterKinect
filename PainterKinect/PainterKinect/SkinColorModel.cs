using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using OpenCvSharp;

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

		public void DetectSkinRegion( CvMat rgbImage, float colorThreshold, int areaThreshold )
		{
			CvScalar pVal;
			int areaCnt = 0;

			for ( int dy = 0 ; dy < rgbImage.GetSize().Height ; dy++ )
			{
				for ( int dx = 0 ; dx < rgbImage.GetSize().Width ; dx++ )
				{
					pVal = rgbImage.Get2D( dy, dx );

					float nVal = this.colorModelData[(int)pVal.Val0 + 256 * (int)pVal.Val1 + 256 * 256 * (int)pVal.Val2];

					if ( nVal >= colorThreshold )
					{
						rgbImage[dy, dx, 0] = rgbImage[dy, dx, 1] = rgbImage[dy, dx, 2] = 100;
						areaCnt++;
					}
					else
					{
						rgbImage[dy, dx, 0] = rgbImage[dy, dx, 1] = rgbImage[dy, dx, 2] = 0;
					}
				}
			}

			rgbImage.Erode( rgbImage );
			rgbImage.Dilate( rgbImage );
		}
	}
}
