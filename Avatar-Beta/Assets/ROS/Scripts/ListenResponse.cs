using ROSBridgeLib;
using ROSBridgeLib.std_msgs;
using ROSBridgeLib.avatar_msgs;
using SimpleJSON;
using UnityEngine;
using System;


public class ListenResponse
{




	public new static void ServiceCallBack(string service, string msg)
	{
		Debug.Log("Service callback " + service + " " + msg);

		GameObject obj = GameObject.Find("UMA avatar");

		// AudioSource ass = obj.GetComponent<AudioSource>();
		JSONNode node = JSONNode.Parse(msg);
		Avatar avatar = obj.GetComponent<Avatar>();
		avatar.DoServiceCallback(service, node);
	}
}
