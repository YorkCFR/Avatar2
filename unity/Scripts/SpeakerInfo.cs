using ROSBridgeLib;
using ROSBridgeLib.std_msgs;
using ROSBridgeLib.avatar_msgs;
using SimpleJSON;
using UnityEngine;
using System;


public class SpeakerInfo : ROSBridgeSubscriber
{

	public new static string GetMessageTopic()
	{
		return "/avatar2/speaker_info";
	}

	public new static string GetMessageType()
	{
		return "avatar2_interfaces/msg/SpeakerInfo";
	}

	public new static ROSBridgeMsg ParseMessage(JSONNode msg)
	{
		return new SpeakerInfoMsg(msg);
	}


	public new static void CallBack(ROSBridgeMsg msg)
	{
		//Debug.Log("Render callback for speaker info " + msg);

		SpeakerInfoMsg x = (SpeakerInfoMsg)msg;

		/*
		GameObject obj = GameObject.Find("UMA avatar");
		AudioSource ass = obj.GetComponent<AudioSource>();
		ass.clip = clip;
		Avatar avatar = obj.GetComponent<Avatar>();
		avatar.waiting2play = true;
		*/
		Debug.Log("SpeakerInfo Msg obtained");
		Debug.Log("row is " + x.GetRow() + " col is " + x.GetCol());




	}
}