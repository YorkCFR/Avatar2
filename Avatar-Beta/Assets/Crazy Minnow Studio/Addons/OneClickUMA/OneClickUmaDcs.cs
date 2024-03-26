using UnityEngine;

namespace CrazyMinnow.SALSA.OneClicks
{
	public class OneClickUmaDcs : OneClickBase
	{
		/// <summary>
		/// RELEASE NOTES:
		///		See OneClickUmaDcsEditor.cs
		///
		///		legacy notes:
		///		2.5.1 (2021-01-21)
		///			+ Added SilenceAnalyzer implementation by default.
		/// 	2.1.3 (2019-08-27)
		/// 		+ support for non-standard dynamic setup. Added Inspector option
		/// 			to disable dynamic setup and a public UmaUepDriver method
		/// 			to manually start initialization:
		/// 				UmaUepDriver.ManualStart(UMAExpressionPlayer)
		///		2.0.0-BETA : Initial release.
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
		public static void Setup(GameObject gameObject)
		{
			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//	SETUP Requirements:
			//		use NewExpression("expression name") to start a new viseme/emote expression.
			//		use AddShapeComponent to add blendshape configurations, passing:
			//			- string array of shape names to look for.
			//			  : string array can be a single element.
			//			  : string array can be a single regex search string.
			//			    note: useRegex option must be set true.
			//			- optional string name prefix for the component.
			//			- optional blend amount (default = 1.0f).
			//			- optional regex search option (default = false).

			Init();

			#region SALSA-Configuration
			NewConfiguration(OneClickConfiguration.ConfigType.Salsa);
			{
				////////////////////////////////////////////////////////
				// SMR regex searches (enable/disable/add as required).

				////////////////////////////////////////////////////////
				// Adjust SALSA settings to taste...
				// - data analysis settings
				autoAdjustAnalysis = true;
				autoAdjustMicrophone = false;

				// - advanced dynamics settings
				loCutoff = 0.03f;
				hiCutoff = 0.75f;
				useAdvDyn = true;
				advDynPrimaryBias = 0.5f;
				useAdvDynJitter = true;
				advDynJitterAmount = 0.1f;
				advDynJitterProb = 0.25f;
				advDynSecondaryMix = 0f;
				emphasizerTrigger = 0.15f;

				////////////////////////////////////////////////////////
				// Viseme setup...

				NewExpression("w");
				AddUepPoseComponent("jawOpen_Close", 0.08f, 0f, 0.06f, "jawOpen_Close", 0.09f);
				AddUepPoseComponent("mouthNarrow_Pucker", 0.08f, 0f, 0.06f, "mouthNarrow_Pucker", -1f);

				NewExpression("t");
				AddUepPoseComponent("leftUpperLipUp_Down", 0.08f, 0f, 0.06f, "leftUpperLipUp_Down", 0.72f);
				AddUepPoseComponent("rightUpperLipUp_Down", 0.08f, 0f, 0.06f, "rightUpperLipUp_Down", 0.62f);
				AddUepPoseComponent("tongueUp_Down", 0.08f, 0f, 0.06f, "tongueUp_Down", 0.49f);
				AddUepPoseComponent("mouthUp_Down", 0.08f, 0f, 0.06f, "mouthUp_Down", -0.44f);
				AddUepPoseComponent("mouthNarrow_Pucker", 0.08f, 0f, 0.06f, "mouthNarrow_Pucker", 0.4f);
				AddUepPoseComponent("leftMouthSmile_Frown", 0.08f, 0f, 0.06f, "leftMouthSmile_Frown", 0.42f);
				AddUepPoseComponent("rightMouthSmile_Frown", 0.08f, 0f, 0.06f, "rightMouthSmile_Frown", 0.42f);
				AddUepPoseComponent("leftLowerLipUp_Down", 0.08f, 0f, 0.06f, "leftLowerLipUp_Down", -0.37f);
				AddUepPoseComponent("rightLowerLipUp_Down", 0.08f, 0f, 0.06f, "rightLowerLipUp_Down", -0.37f);

				NewExpression("f");
				AddUepPoseComponent("jawOpen_Close", 0.08f, 0f, 0.06f, "jawOpen_Close", 0.02f);
				AddUepPoseComponent("leftLowerLipUp_Down", 0.08f, 0f, 0.06f, "leftLowerLipUp_Down", 0.68f);
				AddUepPoseComponent("rightLowerLipUp_Down", 0.08f, 0f, 0.06f, "rightLowerLipUp_Down", 0.64f);
				AddUepPoseComponent("leftUpperLipUp_Down", 0.08f, 0f, 0.06f, "leftUpperLipUp_Down", 0.53f);
				AddUepPoseComponent("rightUpperLipUp_Down", 0.08f, 0f, 0.06f, "rightUpperLipUp_Down", 0.47f);
				AddUepPoseComponent("leftMouthSmile_Frown", 0.08f, 0f, 0.06f, "leftMouthSmile_Frown", 0.24f);
				AddUepPoseComponent("rightMouthSmile_Frown", 0.08f, 0f, 0.06f, "rightMouthSmile_Frown", 0.24f);

				NewExpression("th");
				AddUepPoseComponent("jawOpen_Close", 0.08f, 0f, 0.06f, "jawOpen_Close", 0.1f);
				AddUepPoseComponent("mouthNarrow_Pucker", 0.08f, 0f, 0.06f, "mouthNarrow_Pucker", -0.49f);
				AddUepPoseComponent("leftUpperLipUp_Down", 0.08f, 0f, 0.06f, "leftUpperLipUp_Down", 0.66f);
				AddUepPoseComponent("rightUpperLipUp_Down", 0.08f, 0f, 0.06f, "rightUpperLipUp_Down", 0.53f);
				AddUepPoseComponent("mouthUp_Down", 0.08f, 0f, 0.06f, "mouthUp_Down", 0.19f);
				AddUepPoseComponent("tongueUp_Down", 0.08f, 0f, 0.06f, "tongueUp_Down", 0.43f);

				NewExpression("ow");
				AddUepPoseComponent("jawOpen_Close", 0.08f, 0f, 0.06f, "jawOpen_Close", 0.32f);
				AddUepPoseComponent("mouthNarrow_Pucker", 0.08f, 0f, 0.06f, "mouthNarrow_Pucker", -1f);
				AddUepPoseComponent("mouthUp_Down", 0.08f, 0f, 0.06f, "mouthUp_Down", 0.39f);

				NewExpression("ee");
				AddUepPoseComponent("leftMouthSmile_Frown", 0.08f, 0f, 0.06f, "leftMouthSmile_Frown", 0.87f);
				AddUepPoseComponent("rightMouthSmile_Frown", 0.08f, 0f, 0.06f, "rightMouthSmile_Frown", 0.87f);
				AddUepPoseComponent("leftLowerLipUp_Down", 0.08f, 0f, 0.06f, "leftLowerLipUp_Down", -0.9f);
				AddUepPoseComponent("rightLowerLipUp_Down", 0.08f, 0f, 0.06f, "rightLowerLipUp_Down", -0.76f);
				AddUepPoseComponent("leftUpperLipUp_Down", 0.08f, 0f, 0.06f, "leftUpperLipUp_Down", 0.4f);
				AddUepPoseComponent("rightUpperLipUp_Down", 0.08f, 0f, 0.06f, "rightUpperLipUp_Down", 0.4f);
				AddUepPoseComponent("mouthUp_Down", 0.08f, 0f, 0.06f, "mouthUp_Down", -0.54f);

				NewExpression("oo");
				AddUepPoseComponent("jawOpen_Close", 0.08f, 0f, 0.06f, "jawOpen_Close", 0.58f);
				AddUepPoseComponent("mouthNarrow_Pucker", 0.08f, 0f, 0.06f, "mouthNarrow_Pucker", -1f);
				AddUepPoseComponent("mouthUp_Down", 0.08f, 0f, 0.06f, "mouthUp_Down", 0.79f);
				AddUepPoseComponent("leftMouthSmile_Frown", 0.08f, 0f, 0.06f, "leftMouthSmile_Frown", 0.5f);
				AddUepPoseComponent("rightMouthSmile_Frown", 0.08f, 0f, 0.06f, "rightMouthSmile_Frown", 0.5f);
				AddUepPoseComponent("tongueUp_Down", 0.08f, 0f, 0.06f, "tongueUp_Down", 0.45f);
			}
			#endregion // SALSA-configuration

			#region EmoteR-Configuration
			NewConfiguration(OneClickConfiguration.ConfigType.Emoter);
			{
				////////////////////////////////////////////////////////
				// SMR regex searches (enable/disable/add as required).

				useRandomEmotes = true;
				isChancePerEmote = true;
				numRandomEmotesPerCycle = 0;
				randomEmoteMinTimer = 1f;
				randomEmoteMaxTimer = 2f;
				randomChance = 0.5f;
				useRandomFrac = false;
				randomFracBias = 0.5f;
				useRandomHoldDuration = false;
				randomHoldDurationMin = 0.1f;
				randomHoldDurationMax = 0.5f;

				////////////////////////////////////////////////////////
				// Emote setup...

				NewExpression("exasper");
				AddEmoteFlags(false, true, false, 1f);
				AddUepPoseComponent("leftCheekPuff_Squint", 0.25f, 0.15f, 0.2f, "leftCheekPuff_Squint", 0.28f);
				AddUepPoseComponent("rightCheekPuff_Squint", 0.25f, 0.15f, 0.2f, "rightCheekPuff_Squint", 0.28f);
				AddUepPoseComponent("midBrowUp_Down", 0.25f, 0.15f, 0.2f, "midBrowUp_Down", 0.99f);

				NewExpression("soften");
				AddEmoteFlags(false, true, false, 1f);
				AddUepPoseComponent("leftMouthSmile_Frown", 0.25f, 0.15f, 0.2f, "leftMouthSmile_Frown", 0.5f);
				AddUepPoseComponent("rightMouthSmile_Frown", 0.25f, 0.15f, 0.2f, "rightMouthSmile_Frown", 0.5f);
				AddUepPoseComponent("leftBrowUp_Down", 0.25f, 0.15f, 0.2f, "leftBrowUp_Down", 0.76f);
				AddUepPoseComponent("rightBrowUp_Down", 0.25f, 0.15f, 0.2f, "rightBrowUp_Down", 0.76f);

				NewExpression("browsUp");
				AddEmoteFlags(false, true, false, 1f);
				AddUepPoseComponent("leftBrowUp_Down", 0.2f, 0.1f, 0.25f, "leftBrowUp_Down", 0.87f);
				AddUepPoseComponent("rightBrowUp_Down", 0.25f, 0.15f, 0.2f, "rightBrowUp_Down", 1f);
				AddUepPoseComponent("midBrowUp_Down", 0.15f, 0.2f, 0.15f, "midBrowUp_Down", 0.9f);

				NewExpression("browUp");
				AddEmoteFlags(false, true, false, 1f);
				AddUepPoseComponent("leftBrowUp_Down", 0.25f, 0.15f, 0.2f, "leftBrowUp_Down", 0.49f);
				AddUepPoseComponent("rightBrowUp_Down", 0.25f, 0.15f, 0.2f, "rightBrowUp_Down", 1f);
				AddUepPoseComponent("midBrowUp_Down", 0.25f, 0.15f, 0.2f, "midBrowUp_Down", 0.54f);

				NewExpression("squint");
				AddEmoteFlags(false, true, false, 1f);
				AddUepPoseComponent("leftCheekPuff_Squint", 0.25f, 0.15f, 0.2f, "leftCheekPuff_Squint", -0.41f);
				AddUepPoseComponent("rightCheekPuff_Squint", 0.25f, 0.15f, 0.2f, "rightCheekPuff_Squint", -0.41f);
				AddUepPoseComponent("midBrowUp_Down", 0.25f, 0.15f, 0.2f, "midBrowUp_Down", -0.95f);

				NewExpression("focus");
				AddEmoteFlags(false, true, false, 1f);
				AddUepPoseComponent("leftCheekPuff_Squint", 0.25f, 0.15f, 0.2f, "leftCheekPuff_Squint", -0.41f);
				AddUepPoseComponent("rightCheekPuff_Squint", 0.25f, 0.15f, 0.2f, "rightCheekPuff_Squint", -0.41f);

				NewExpression("flare");
				AddEmoteFlags(false, true, false, 1f);
				AddUepPoseComponent("noseSneer", 0.25f, 0.15f, 0.2f, "noseSneer", 0.404f);
				AddUepPoseComponent("midBrowUp_Down", 0.25f, 0.15f, 0.2f, "midBrowUp_Down", 0.44f);

				NewExpression("scrunch");
				AddEmoteFlags(false, true, false, 1f);
				AddUepPoseComponent("midBrowUp_Down", 0.25f, 0.15f, 0.2f, "midBrowUp_Down", -0.97f);
				AddUepPoseComponent("noseSneer", 0.25f, 0.15f, 0.2f, "noseSneer", 0.331f);
				AddUepPoseComponent("leftCheekPuff_Squint", 0.25f, 0.15f, 0.2f, "leftCheekPuff_Squint", -0.27f);
				AddUepPoseComponent("rightCheekPuff_Squint", 0.25f, 0.15f, 0.2f, "rightCheekPuff_Squint", -0.27f);
			}
			#endregion // EmoteR-configuration

			DoOneClickiness(gameObject);

			if (selectedObject.GetComponent<SalsaAdvancedDynamicsSilenceAnalyzer>() == null)
				selectedObject.AddComponent<SalsaAdvancedDynamicsSilenceAnalyzer>();
		}
	}
}
