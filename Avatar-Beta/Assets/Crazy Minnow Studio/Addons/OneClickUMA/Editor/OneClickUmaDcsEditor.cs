using UnityEditor;
using UnityEngine;

namespace CrazyMinnow.SALSA.OneClicks
{
	/// <summary>
	/// RELEASE NOTES:
	///		2.7.2 (2023-09-28):
	///			REQUIRES: OneClickBase and Base core files v2.7.2+, does NOT work with prior versions.
	///			REMOVED prefab breakdown dependency.
	///			~ QueueProcessor configuration now called from Editor script.
	///			~ AudioSource configuration now called from Editor script.
	///
	///			PACKAGE CONTAINS:
	///				OneClickUmaDcsEditor.cs
	///				UmaUepDriverEditor.cs
	///				OneClickUmaDcs.cs
	///				OneClickUmaDcsEyes.cs
	///				UmaUepDriver.cs
	///				UmaUepDriverEditorPreview.cs
	/// 
	///		2.5.1 (2021-01-21):
	///			+ Updated for and requires UMA v2.11.5+
	///			+ SALSA/EmoteR Preview now works with in-Editor UMA Dynamic Character Avatar.
	///			~ SilenceAnalyzer now added by default.
	/// 	2.5.0 (2020-07-20):
	/// 		+ Support for SALSA Suite Dynamic Influence Detection.
	/// 		~ REQUIRES: SALSA LipSync Suite v2.5.0+
	/// 		~ UmaUepDriver animations are greatly improved; smoother and more accurate.
	/// 		! UmaUepDriver now properly drives shape interruptions.
	/// 		! Now works with UMA 2.10
	/// 		! No longer errors for Eyes module when Preview is enabled.
	///		2.3.0 (2020-02-02):
	/// 		~ updated to operate with SALSA Suite v2.3.0+
	/// 		NOTE:
	/// 			1. Does not work with prior versions of SALSA Suite (before v2.3.0).
	/// 			2. For proper operation, disable 'Merge with Influencer' in QueueProcessor
	/// 			settings.
	/// 			3. Lid tracking is configured but not displayed in this version. This will be
	/// 			addressed in a future version.
	/// 	2.1.0 (2019-08-25):
	/// 		+ Experimental support for non-dynamic UMA avatars (assumes a
	/// 			correctly configured UMAExpressionPlayer).
	/// 	2.0.0 (2019-07-20):
	/// 		- confirmed operation with Base 2.1.5
	///			+ Initial release.
	/// ==========================================================================
	/// PURPOSE: This script provides simple, simulated lip-sync input to the
	///		Salsa component from text/string values. For the latest information
	///		visit crazyminnowstudio.com.
	/// ==========================================================================
	/// DISCLAIMER: While every attempt has been made to ensure the safe content
	///		and operation of these files, they are provided as-is, without
	///		warranty or guarantee of any kind. By downloading and using these
	///		files you are accepting any and all risks associated and release
	///		Crazy Minnow Studio, LLC of any and all liability.
	/// ==========================================================================
	/// </summary>
	public class OneClickUmaDcsEditor : MonoBehaviour
	{
		[MenuItem("GameObject/Crazy Minnow Studio/SALSA LipSync/One-Clicks/UMA DCS")]
		public static void UmaDcsSetup()
		{
			GameObject go = Selection.activeGameObject;
			if (go == null)
			{
				Debug.LogWarning(
				                 "NO OBJECT SELECTED: You must select an object in the scene to apply the OneClick to.");
				return;
			}

			ApplyOneClick(go);
		}

		private static void ApplyOneClick(GameObject go)
		{
			var uepDriver = go.GetComponent<UmaUepDriver>();
			if (uepDriver == null)
				go.AddComponent<UmaUepDriver>();

			var uepProxy = go.GetComponent<UmaUepProxy>();
			if (uepProxy == null)
				go.AddComponent<UmaUepProxy>();

			OneClickUmaDcs.Setup(go);
			OneClickUmaDcsEyes.Setup(go);

			// add QueueProcessor
			OneClickBase.AddQueueProcessor(go);

			// configure AudioSource
			var clip = AssetDatabase.LoadAssetAtPath<AudioClip>(OneClickBase.RESOURCE_CLIP);
			OneClickBase.ConfigureSalsaAudioSource(go, clip, true);
		}
	}
}