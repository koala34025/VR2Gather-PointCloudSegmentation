﻿using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;
using PCLDataProviders;
using Utils;

namespace Workers {
    public class MyRabbitMQReceiver : RabbitMQReceiver
    {
        public MyRabbitMQReceiver(string uri, string exchange)
        {
            ConnectionProperties.ConnectionURI = uri;
            ConnectionProperties.ExchangeName = exchange;
            Debug.Log($"xxxjack MyRabbitMQReceiver: uri={uri}, exchange={exchange}");
        }
    }

    public class CerthReader : BaseWorker   // Doesn't have to be a BaseWorker, but EntityPipeline expects it.
    {
        float voxelSize;
        QueueThreadSafe outQueue;
        QueueThreadSafe out2Queue;

        private PCLIdataProvider dataProvider;  // Connection to RabbitMQ over which normal RGBD data comes in.
        private PCLIdataProvider metaDataProvider;  // Connection to RabbitMQ over which metadata comes in.
        bool metaDataReceived = false;          // Set to true once metadata has been received
        GCHandle metaDataHandle;                // Set to unmanaged memory handle where metadata has been stored.
        cwipc.pointcloud mostRecentPc;          // Stores the most recently received pointcloud (if any)
        const int pcl_id = 0;                   // Index of Cert pc constructor (constant for now)

        private RabbitMQReceiver PCLRabbitMQReceiver;
        private RabbitMQReceiver MetaRabbitMQReceiver;

        public CerthReader(Config._User._PCSelfConfig cfg, QueueThreadSafe _outQueue, QueueThreadSafe _out2Queue)
        {
            outQueue = _outQueue;
            out2Queue = _out2Queue;
            voxelSize = cfg.voxelSize;

            if (cfg.CerthReaderConfig == null)
            {
                Debug.LogError("CerthReader: CerthReaderConfig is null");
                return;
            }

            PCLRabbitMQReceiver = new MyRabbitMQReceiver(cfg.CerthReaderConfig.ConnectionURI, cfg.CerthReaderConfig.PCLExchangeName);
            if (PCLRabbitMQReceiver == null)
            {
                Debug.LogError("CerthReader: PCLRabbitMQReceiver is null");
                return;
            }

            MetaRabbitMQReceiver = new MyRabbitMQReceiver(cfg.CerthReaderConfig.ConnectionURI, cfg.CerthReaderConfig.MetaExchangeName);
            if (MetaRabbitMQReceiver == null)
            {
                Debug.LogError("CerthReader: MetaRabbitMQReceiver is null");
                return;
            }

            PCLRabbitMQReceiver.OnDataReceived += OnNewPCLData;
            PCLRabbitMQReceiver.Enabled = true;

            MetaRabbitMQReceiver.OnDataReceived += OnNewMetaData;
            MetaRabbitMQReceiver.Enabled = true;
        }

        public void StopAndWait() {
            Stop();
        }

        public void Stop() {
            PCLRabbitMQReceiver.OnDataReceived -= OnNewPCLData;
            PCLRabbitMQReceiver.Enabled = false;
            MetaRabbitMQReceiver.OnDataReceived -= OnNewMetaData;
            MetaRabbitMQReceiver.Enabled = false;

            Debug.Log("PCCerthReader: Stopped.");
        }

        // Informing that the metadata were received
        private void OnNewMetaData(object sender, EventArgs<byte[]> e) {
            lock (e) {
                if (e.Value != null && !metaDataReceived) {
                    var buffer = e.Value; // Buffer 's data
                    metaDataHandle = GCHandle.Alloc(buffer, GCHandleType.Pinned); // GCHandler for the buffer
                    var pnt = metaDataHandle.AddrOfPinnedObject(); // Buffer 's address
                    metaDataReceived = native_pointcloud_receiver_pinvoke.received_metadata(pnt, buffer.Length, pcl_id);
                }
            }
        }

        // Updating the pointcloud every time a new buffer is received from the network
        private void OnNewPCLData(object sender, EventArgs<byte[]> e) {
            lock (e) {
                if (e.Value == null) {
                    Debug.LogWarning("CerthReader: OnNewPCLData: received null data");
                    return;
                }

                if (!metaDataReceived) {
                    Debug.Log("CerthReader: OnNewPCLData: received data, but no metadata yet");
                    return;

                }
                if (outQueue.Count < 2) { // FPA_TODO: Fix this using queue.Size
                    // Flaging that a new buffer is received
                    var buffer = e.Value; // Buffer 's data
                    GCHandle rgbdHandle = GCHandle.Alloc(buffer, GCHandleType.Pinned); // GCHandler for the buffer
                    System.IntPtr rgbdPtr = rgbdHandle.AddrOfPinnedObject(); // Buffer 's address
                    System.IntPtr pclPtr = native_pointcloud_receiver_pinvoke.callColorizedPCloudFrameDLL(rgbdPtr, buffer.Length, pcl_id); // Pointer of the returned structure
                    if (pclPtr == System.IntPtr.Zero)
                    {
                        Debug.LogWarning("CerthReader: callColorizedPCloudFrameDLL returned NULL");
                        return;
                    }
                    float[] bbox = { -1f, 1f, -1f, 0.5f, -3f, 1f };
                    System.UInt64 timestamp = 0;
                    cwipc.pointcloud pc = cwipc.from_certh(pclPtr, bbox, timestamp);
                    if (pc == null)
                    {
                        Debug.LogWarning("CerthReader: cwipc.from_certh did not produce a pointcloud");
                    }
                    else
                    {
                        outQueue.Enqueue(pc);
                        out2Queue.Enqueue(pc);

                    }
                    // Freeing the GCHandler
                    rgbdHandle.Free();
                }
            }
        }
    }
}