using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;

namespace PainterKinect
{
	class Logging
	{
		// File Name
		private static string LOG_FILE = @"logfile.html";

		// Stream Writer
		private static StreamWriter writer;

		// Initialized Status
		private static bool isInitialized = false;

		// Writer
		public static void InitializeLogging()
		{
			// Setup Logging
			writer = new StreamWriter( Path.GetFullPath( LOG_FILE ), true, System.Text.Encoding.UTF8 );

			// Set Auto Flush
			writer.AutoFlush = true;

			// Check Initialize Status
			if ( writer != null )
			{
				isInitialized = true;
				writer.WriteLine( "<hr>" );
				PrintLog( "Logging", "Logging Module Initialized." );
			}
		}

		public static void CloseLogging()
		{
			// Flush Stream
			writer.Flush();

			// Close Stream
			writer.Close();

			// Unset Initialize Status
			isInitialized = false;
		}

		public static void PrintDebugLog( string tag, string msg )
		{
			if ( isInitialized )
				writer.WriteLine( "<font color='purple'><b>[DEBUG|" + tag + "]</b> : " + msg + "</font><br>" );
			else
				Console.WriteLine( "[DEBUG|" + tag + "] : " + msg );
		}

		public static void PrintErrorLog( string tag, string msg )
		{
			if ( isInitialized )
				writer.WriteLine( "<font color='red'><b>[ERROR|" + tag + "]</b> : " + msg + "</font><br>" );
			else
				Console.WriteLine( "[ERROR|" + tag + "] : " + msg );
		}

		public static void PrintLog( string tag, string msg )
		{
			if ( isInitialized )
				writer.WriteLine( "<font color='black'><b>[LOG|" + tag + "]</b> : " + msg + "</font><br>" );
			else
				Console.WriteLine( "[LOG|" + tag + "] : " + msg );
		}

	}
}
