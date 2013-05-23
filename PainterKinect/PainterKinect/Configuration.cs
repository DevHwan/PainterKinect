using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PainterKinect
{
	public class Configuration
	{
		// Hand Region Size
		public static int HAND_REGION_WIDTH = 150;
		public static int HAND_REGION_HEIGHT = 150;

		// Object Region Size
		public static int OBJECT_REGION_WIDTH = 50;
		public static int OBJECT_REGION_HEIGHT = 50;

		// Screen Size
		public static double  SCREEN_WIDTH = 800.0;
		public static double SCREEN_HEIGHT = 600.0;
		public static double SCREEN_ASPECT = SCREEN_WIDTH / SCREEN_HEIGHT;
	}
}
