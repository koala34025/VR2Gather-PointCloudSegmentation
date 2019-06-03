﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PointCloudsMainController : MonoBehaviour {
    // Start is called before the first frame update
    void Start() {
        foreach( var pc in Config.Instance.PCs) {
            PointCloudBaseRenderer pct = null;
            if (pc.forceMesh || SystemInfo.graphicsShaderLevel < 50) 
                pct = new GameObject("PC").AddComponent<PointCloudMeshRenderer>();
            else            
                pct = new GameObject("PC").AddComponent<PointCloudBufferRenderer>();
            pct.Init(pc);
            pct.transform.parent = transform;
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}