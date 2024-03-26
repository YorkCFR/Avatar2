using SimpleJSON;
using UnityEngine;

/**
 * Define a Header message. These have been hand-crafted from the corresponding msg file.
 * 
 * Version History
 * 4.0 - changes to make it ROS2 compatable
 * 3.3 - updated to most recent version
 * 3.1 - changed methods to start with an upper case letter to be more consistent with c# style.
 * 3.0 - modification from hand crafted version 2.0
 * 
 * @author Michael Jenkin, Robert Codd-Downey and Andrew Speers
 * @version 4.0
 */

namespace ROSBridgeLib {
	namespace std_msgs {
		public class HeaderMsg : ROSBridgeMsg {
			private TimeMsg _stamp;
			private string _frame_id;
			
			public HeaderMsg(JSONNode msg) {
				Debug.Log("Header message being parsed " + msg);
				_stamp = new TimeMsg (msg ["stamp"]);
				_frame_id = msg["frame_id"];
			}
			
			public HeaderMsg(TimeMsg stamp, string frame_id) {
				_stamp = stamp;
				_frame_id = frame_id;
			}
			
			public static string GetMessageType() {
				return "std_msgs/Header";
			}
			
			public TimeMsg GetTimeMsg() {
				return _stamp;
			}

			public string GetFrameId() {
				return _frame_id;
			}
			
			public override string ToString() {
				return "Header [stamp=" + _stamp + ", frame_id='" + _frame_id + "']";
			}
			
			public override string ToYAMLString() {
				return "{\"stamp\" : " + _stamp.ToYAMLString () + ", frame_id='" + _frame_id + "'}";
			}
		}
	}
}
