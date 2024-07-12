using SimpleJSON;
using UnityEngine;
using ROSBridgeLib.std_msgs;


/**
 * Define an Audio message
 * Version History
 *
 *  V1.0 - hand crafted from the ros2 message
 * 
 */

namespace ROSBridgeLib
{
	namespace avatar_msgs
	{
		public class SpeakerInfoMsg : ROSBridgeMsg
		{
			private int _seq;
			private HeaderMsg _header;
			private float _row;
			private float _col;
			private float _width;
			private float _height;
			private StringMsg _info;
			// note: SensorMsg/Image face not currently processed

			public SpeakerInfoMsg(JSONNode msg)
			{
				Debug.Log("SpeakerInfo message being parsed");
				_header = new HeaderMsg(msg["header"]);
				_seq = msg["seq"].AsInt;
				_row = msg["row"].AsFloat;
				_col = msg["col"].AsFloat;
				_width = msg["width"].AsFloat;
				_height = msg["height"].AsFloat;
				_info = new StringMsg(msg["info"]);
				Debug.Log("All done");
			}

			public static string GetMessageType()
			{
				return "avatar2_interfaces/SpeakerInfo";
			}


			public SpeakerInfoMsg(HeaderMsg header, int seq, float row, float col, float width, float height, StringMsg info)
			{
				_seq = seq;
				_header = header;
				_row = row;
				_col = col;
				_width = width;
				_height = height;
				_info = info;
			}


			public HeaderMsg GetHeader()
			{
				return _header;
			}

			public int GetSeq()
			{
				return _seq;
			}

			public float GetRow()
			{
				return _row;
			}

			public float GetCol()
			{
				return _col;
			}

			public float GetWidth()
			{
				return _width;
			}

			public float GetHeight()
			{
				return _height;
			}

			public StringMsg GetInfo()
            {
				return _info;
            }

			public override string ToString()
			{
				return "avatar2_interfaces/Spekaerinfo[header=" + _header + ",  seq=" + _seq + ", row=" + _row + ", _col=" + _col + " _width=" + _width + " _height=" + _height + " +info=" + _info + "]";
			}

			public override string ToYAMLString()
			{
				return "{\"header\" : " + _header + ", \"seq\" : " + _seq + ", \"row\" : " + _row + ", \"col\" : " + _col + ", \"width\" : " + _width + ", \"height\": " + _height + ", \"info\": " + _info + "}";
			}

		}
	}
}

