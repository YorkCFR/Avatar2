using SimpleJSON;
using UnityEngine;
using ROSBridgeLib.std_msgs;


/**
 * Define an AudioInput message
 * Version History
 *
 *  V1.0 - hand crafted from the ros2 message
 * 
 */

namespace ROSBridgeLib
{
	namespace avatar_msgs
	{
		public class AudioOutputMsg : ROSBridgeMsg
		{
			private int _seq;
			private string _audio;
			private HeaderMsg _header;

			public AudioOutputMsg(JSONNode msg)
			{
				Debug.Log("Audio Input message being parsed " + msg);
				_header = new HeaderMsg(msg["header"]);
				Debug.Log("Header is good");
				_seq = msg["audio_sequence_number"].AsInt;
				Debug.Log("got sequence number");

				_audio = msg["audio"];
				Debug.Log("All done");
			}

			public static string GetMessageType()
			{
				return "avatar2_interfaces/AudioOutput";
			}


			public AudioOutputMsg(HeaderMsg header, int seq, string audio)
			{
				_seq = seq;
				_header = header;
				_audio = audio;
			}


			public HeaderMsg GetHeader()
			{
				return _header;
			}

			public int GetSeq()
			{
				return _seq;
			}

			public string GetAudio()
			{
				return _audio;
			}

			public override string ToString()
			{
				return "avatar2_interfaces/AudioOutput[header=" + _header + ",  seq=" + _seq + ", aduio=" + _audio + "]";
			}

			public override string ToYAMLString()
			{
				return "{\"header\" : " + _header + ", \"seq\" : " + _seq + ", \"audio\" : " + _audio + "}";
			}

		}
	}
}