using ROSBridgeLib;
using ROSBridgeLib.std_msgs;
using ROSBridgeLib.avatar_msgs;
using SimpleJSON;
using UnityEngine;
using System;


public class AvatarAudio : ROSBridgeSubscriber
{

	public new static string GetMessageTopic()
	{
		return "/avatar2/out_raw_audio";
	}

	public new static string GetMessageType()
	{
		return "avatar2_interfaces/msg/Audio";
	}

	public new static ROSBridgeMsg ParseMessage(JSONNode msg)
	{
		return new AudioMsg(msg);
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
		Debug.Log("Render callback for avatar audio " + msg);

		AudioMsg x = (AudioMsg)msg;

		byte[] wav = StringToByteArray(x.GetAudio());
		Debug.Log("Assuming 16 bits signed audio file is " + (wav.Length - 44) / 2 + " samples long");


		AudioClip clip = null;
		if(x.GetFormat() == "WAV_1_44100")
		    clip = AudioClip.Create("speak", (wav.Length - 44) / 2, 1, 44100, false);
		else if(x.GetFormat() == "WAV_1_22050")
			clip = AudioClip.Create("speak", (wav.Length - 44) / 2, 1, 22050, false);
		else
        {
			Debug.Log("Unknown audio format " + x.GetFormat());
			return;
        }
		clip.SetData(WavtoFloatArray(wav), 0);

        Debug.Log("Cueing up audio to play when we can");
		GameObject obj = GameObject.Find("UMA avatar");
		AudioSource ass = obj.GetComponent<AudioSource>();
		ass.clip = clip;
		Avatar avatar = obj.GetComponent<Avatar>();
		avatar.waiting2play = true;
		Debug.Log("All ready to play");




	}
}
