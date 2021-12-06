﻿#if UNITY_EDITOR
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;
using UnityEngine;
using System.IO;
using VRT.Core;

public class CopyConfigOnBuild : IPostprocessBuildWithReport
{
    public int callbackOrder {  get { return 0; } }
    public void OnPostprocessBuild(BuildReport report)
    {
        string srcDir = VRT.Core.Config.ConfigFilename("") + "/";
        string dstDir;
        
        if (report.summary.platform == BuildTarget.StandaloneOSX)
        {
            // For Mac the outputPath is the application bundle path. We want to store config files inside the Contens folder.
            dstDir = report.summary.outputPath + "/Contents/";
        } else
        {
            // For Windows and Linux this is "good enough": the config files should be in the same folder as the executable
            // For other platforms this will probably need work.
            dstDir = Path.GetDirectoryName(report.summary.outputPath) + "/";
        }
        

        Debug.Log($"CopyConfigOnBuild.OnPostProcessBuild src {srcDir} dst {dstDir}");
        if (File.Exists(srcDir + "config.json"))
        {
            File.Copy(srcDir + "config.json", dstDir + "config.json");
            Debug.Log($"CopyConfigOnBuild.OnPostProcessBuild copied config.json");
        }
        if (File.Exists(srcDir + "cameraconfig.xml"))
        {
            File.Copy(srcDir + "cameraconfig.xml", dstDir + "cameraconfig.xml");
            Debug.Log($"CopyConfigOnBuild.OnPostProcessBuild copied cameraconfig.xml");
        }


    }
}
#endif