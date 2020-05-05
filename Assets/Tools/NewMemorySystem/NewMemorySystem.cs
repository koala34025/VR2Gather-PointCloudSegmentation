﻿#define  TEST_LOCAL_PC
//#define TEST_PC
//#define TEST_VOICECHAT

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class NewMemorySystem : MonoBehaviour
{

    public bool         forceMesh = true;
    public bool         localPCs = false;
    public bool         useVoice = false;
    Workers.BaseWorker  reader;
    Workers.BaseWorker  encoder;
    Workers.BaseWorker  decoder;
    Workers.BaseWorker  dashWriter;
    Workers.BaseWorker  dashReader;

    Workers.BaseWorker  binWriter;
    Workers.BaseWorker  binReader;


    Workers.BaseWorker  preparer;
    QueueThreadSafe     preparerQueue = new QueueThreadSafe();
    QueueThreadSafe     encoderQueue = new QueueThreadSafe();
    QueueThreadSafe     writerQueue = new QueueThreadSafe();
    QueueThreadSafe     decoderQueue = new QueueThreadSafe();
    MonoBehaviour       render;

    // rtmp://127.0.0.1:1935/live/signals
    // Start is called before the first frame update
    void Start() {
        Config._User cfg = Config.Instance.Users[0];
        if (forceMesh) {
            preparer = new Workers.MeshPreparer(preparerQueue);
            render = gameObject.AddComponent<Workers.PointMeshRenderer>();
            ((Workers.PointMeshRenderer)render).preparer = (Workers.MeshPreparer)preparer;
        } else {
            preparer = new Workers.BufferPreparer(preparerQueue);
            render = gameObject.AddComponent<Workers.PointBufferRenderer>();
            ((Workers.PointBufferRenderer)render).preparer = (Workers.BufferPreparer)preparer;
        }


        var pcSelfConfig = cfg.PCSelfConfig;
        if (localPCs)
            reader = new Workers.RS2Reader(pcSelfConfig.RS2ReaderConfig.configFilename, pcSelfConfig.voxelSize, preparerQueue);
        else {
            reader = new Workers.RS2Reader(pcSelfConfig.RS2ReaderConfig.configFilename, pcSelfConfig.voxelSize, encoderQueue);
            encoder = new Workers.PCEncoder(cfg.PCSelfConfig.Encoder.octreeBits, encoderQueue, writerQueue);
            string uuid = System.Guid.NewGuid().ToString();
            var b2d = cfg.PCSelfConfig.Bin2Dash;
            dashWriter = new Workers.B2DWriter("https://vrt-evanescent.viaccess-orca.com/" + uuid + "/", "testBed", b2d.segmentSize, b2d.segmentLife, writerQueue);
            var SUBConfig = cfg.SUBConfig;
            dashReader = new Workers.SUBReader("https://vrt-evanescent.viaccess-orca.com/" + uuid + "/", "testBed", cfg.SUBConfig.streamNumber, cfg.SUBConfig.initialDelay, decoderQueue);
            decoder = new Workers.PCDecoder(decoderQueue, preparerQueue);
        }

        if (useVoice) {
            string uuid = System.Guid.NewGuid().ToString();
            var audioB2D = cfg.PCSelfConfig.AudioBin2Dash;

            gameObject.AddComponent<VoiceDashSender>().Init("https://vrt-evanescent.viaccess-orca.com/" + uuid + "/", "audio", audioB2D.segmentSize, audioB2D.segmentLife); //Audio Pipeline

            var audioConfig = cfg.AudioSUBConfig;
            gameObject.AddComponent<VoiceDashReceiver>().Init("https://vrt-evanescent.viaccess-orca.com/" + uuid + "/", "audio", audioConfig.streamNumber, audioConfig.initialDelay); //Audio Pipeline

        }
    }

    void OnDestroy() {
        dashWriter?.StopAndWait();
        dashReader?.StopAndWait();
        binWriter?.StopAndWait();
        binReader?.StopAndWait();

        reader?.StopAndWait();
        encoder?.StopAndWait();
        decoder?.StopAndWait();
        preparer?.StopAndWait();
        Debug.Log($"NewMemorySystem: Queues references counting: preparerQueue {preparerQueue.Count} encoderQueue {encoderQueue.Count} writerQueue {writerQueue.Count} decoderQueue {decoderQueue.Count}");
        BaseMemoryChunkReferences.ShowTotalRefCount();
    }
}
