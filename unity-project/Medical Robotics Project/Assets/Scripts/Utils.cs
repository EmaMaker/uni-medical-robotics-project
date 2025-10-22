using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Utils : MonoBehaviour
{
    public static string FingerDistalFromName(string n)
    {
        n = n.ToLower();
        if (n.Contains("thumbdistal")) return "Thumb";
        if (n.Contains("indexdistal")) return "Index";
        if (n.Contains("middledistal")) return "Middle";
        if (n.Contains("ringdistal")) return "Ring";
        if (n.Contains("pinkydistal") || n.Contains("littledistal")) return "Pinky";
        if (n.Contains("wrist")) return "Palm";
        return "unknown";
    }
}
