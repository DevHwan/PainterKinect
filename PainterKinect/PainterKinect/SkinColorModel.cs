﻿using System;
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

			// Area Count
			int areaCnt = 0;

			// Image Data Ptr

			CvScalar dat;

			for ( int dy = 0 ; dy < rgbImage.Height ; dy++ )
			{
				for ( int dx = 0 ; dx < rgbImage.Width ; dx++ )
				{
					dat = rgbImage.Get2D( dy, dx );

					float nVal = this.colorModelData[(int)dat.Val0 + 256 * (int)dat.Val1 + 256 * 256 * (int)dat.Val2 ];

					if ( nVal >= colorThreshold )
					{
						//rgbImage.Set2D( dy, dx, new CvScalar( 255, 255, 255 ) );
						areaCnt++;
					}
					else
					{
						rgbImage.Set2D( dy, dx, new CvScalar( 0, 0, 0 ) );
					}
				}
			}

			Cv.Erode( rgbImage, rgbImage );
			Cv.Dilate( rgbImage, rgbImage );

			if ( areaCnt > areaThreshold )
				return 1;
			else
				return 0;
		}
	}
}
