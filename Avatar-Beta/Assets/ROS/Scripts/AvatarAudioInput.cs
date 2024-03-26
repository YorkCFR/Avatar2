using ROSBridgeLib;
using ROSBridgeLib.std_msgs;
using ROSBridgeLib.avatar_msgs;
using SimpleJSON;
using UnityEngine;
using System;


public class AvatarAudioInput : ROSBridgeSubscriber
{

	public new static string GetMessageTopic()
	{
		return "/avatar2/audio_input";
	}

	public new static string GetMessageType()
	{
		return "avatar2_interfaces/msg/AudioInput";
	}

	public new static ROSBridgeMsg ParseMessage(JSONNode msg)
	{
		return new AudioInputMsg(msg);
	}

	public static byte[] StringToByteArray(string hex)
	{
		int NumberChars = hex.Length;
		byte[] bytes = new byte[NumberChars / 2];
		for (int i = 0; i < NumberChars; i += 2)
			bytes[i / 2] = Convert.ToByte(hex.Substring(i, 2), 16);
		return bytes;
	}

	public static float[] WavtoFloatArray(byte[] wav)
    {
		int nsamples = (wav.Length - 44) / 2;
		short[] samples = new short[nsamples];
		float[] outv = new float[nsamples];
		Buffer.BlockCopy(wav, 44, samples, 0, samples.Length * 2);  // drop header and make a signed array of shorts

		for(int i = 0; i < nsamples; i++)
        {
			outv[i] = samples[i] / 32768.0f;
        }
		return outv;
    }


	public new static void CallBack(ROSBridgeMsg msg)
	{
		Debug.Log("Render callback in /Avatar2/audio_input " + msg);

		AudioInputMsg x = (AudioInputMsg)msg;

		byte[] wav = StringToByteArray(x.GetAudio());
		Debug.Log("Assuming 16 bits signed audio file is " + (wav.Length - 44) / 2 + " samples long");



		AudioClip clip = AudioClip.Create("speak", (wav.Length - 44) / 2, 1, 44100, false);
		clip.SetData(WavtoFloatArray(wav), 0);

		GameObject obj = GameObject.Find("UMA avatar");

		AudioSource ass = obj.GetComponent<AudioSource>();
		ass.clip = clip;

		ass.Play();


	}
}