using UnityEngine;
using ROSBridgeLib;
using SimpleJSON;
using ROSBridgeLib.avatar_msgs;


/**
 * This talks to the ROS bridge. There is one constant below that you will probably have to change,
 * the IP address of the ros world. 
 * 
 * @author Michael Jenkin, Walled Khan
 * @version 1.0
 **/

public class Avatar : MonoBehaviour  {
	private ROSBridgeWebSocketConnection ros = null;
	private bool cueAvatar = false;
	private bool listening;
	public bool waiting2play;
	public bool lastSent = true;
	private float playStart;

	// the critical thing here is to define our subscribers, publishers and service response handlers
	void Start () {
		Debug.Log("Connecting to ros");
		ros = new ROSBridgeWebSocketConnection ("ws://192.168.1.41", 9090);
        	ros.AddSubscriber(typeof(AvatarAudio));
		ros.AddServiceResponse(typeof(ListenResponse));
		Debug.Log("And off we go");
		ros.Connect ();
		waiting2play = false;
		StartListening();
	}

	// extremely important to disconnect from ROS. OTherwise packets continue to flow
	void OnApplicationQuit() {
		if(ros!=null)
			ros.Disconnect ();
	}

	public void DoServiceCallback(string service, JSONNode node) {
		listening = node["status"].ToString().Equals("\"true\"");
		Debug.Log("Got the service callback response " + service + " status " + node["status"].ToString()  + " listening " + listening);
    	}

    	public void StopListening() {
		if (!lastSent)
			return;
		Debug.Log("Stop Listening");
		ros.CallService("/avatar2/listen", "avatar2_interfaces/srv/Listen", "{\"listen\": false}");
		lastSent = false;
	}

	void StartListening() {
		if (lastSent)
			return;
		Debug.Log("Start Listening");
		ros.CallService("/avatar2/listen", "avatar2_interfaces/srv/Listen", "{\"listen\": true}");
		lastSent = true;
	}
	

	void Update () {
		float now = Time.realtimeSinceStartup;
		ros.Render ();

		if (now < 5.0) // snooze
		{
		}
		else
		{
            if (!cueAvatar)
            {
				cueAvatar = true;
				lastSent = false; // hack
				StartListening();
				Debug.Log("Starting to listen");
				AudioSource ass = GetComponent<AudioSource>();
				ass.clip = null;
				waiting2play = false;
            }

			//Debug.Log("Listening " + listening + " waiting2play " + waiting2play);
			if (listening)
			{
				if (waiting2play)
				{
					Debug.Log("Waiting to play is true. Signalling to stop listening");
					StopListening();
				} else {
					// Debug.Log("Nothing to play");
				}
			} else {
                if (waiting2play)
                {
					Debug.Log("Not listening and waiting to play. Starting to play");
					AudioSource ass = GetComponent<AudioSource>();
					ass.Play();
					playStart = Time.realtimeSinceStartup;
					waiting2play = false;
				} else {
					float playTime = Time.realtimeSinceStartup - playStart;
					Debug.Log("Playing " + playTime);
					AudioSource audioSource = GetComponent<AudioSource>();
					if ((playTime > 2.0) && !audioSource.isPlaying)
					{
						Debug.Log("Playing is over ");
						StartListening();
					}

				}
			}

         

			//		AudioSource ass = obj.GetComponent<AudioSource>();
			// ass.clip = clip;

			//ass.Play();
			//if (!listening)
			//{
				// AudioSource audioSource = GetComponent<AudioSource>();
				// if (!waiting2play && !audioSource.isPlaying)
				// {
					// StartListening();
					//Debug.Log("Start to listen again");
				//}
			//}
		}
	}
}
