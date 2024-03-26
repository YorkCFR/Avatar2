using UnityEngine;
using ROSBridgeLib;
using ROSBridgeLib.avatar_msgs;


/**
 * This is a toy example of the Unity-ROS interface talking to the TurtleSim 
 * tutorial (circa Groovy). Note that due to some changes since then this will have
 * to be slightly re-written, but as its a test ....
 * 
 * THis does all the ROS work.
 * 
 * @author Michael Jenkin, Robert Codd-Downey and Andrew Speers
 * @version 3.0
 **/

public class Avatar : MonoBehaviour  {
	private ROSBridgeWebSocketConnection ros = null;	


	// the critical thing here is to define our subscribers, publishers and service response handlers
	void Start () {

		Debug.Log("Connecting to ros");
		ros = new ROSBridgeWebSocketConnection ("ws://192.168.0.110", 9090);
		ros.AddSubscriber (typeof(AvatarAudioOutput));
		Debug.Log("And off we go");
		ros.Connect ();

	}

	// extremely important to disconnect from ROS. OTherwise packets continue to flow
	void OnApplicationQuit() {
		if(ros!=null)
			ros.Disconnect ();
	}
	

	void Update () {

		ros.Render ();

	}
}
