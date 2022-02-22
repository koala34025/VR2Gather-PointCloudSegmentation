﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using VRT.Core;

namespace VRT.UserRepresentation.PointCloud
{
    using Timestamp = System.Int64;
    using Timedelta = System.Int64;

    public class PCDecoder : BaseWorker
    {
        protected cwipc.decoder decoder;
        protected QueueThreadSafe inQueue;
        protected QueueThreadSafe outQueue;
        static int instanceCounter = 0;
        int instanceNumber = instanceCounter++;
        bool debugColorize = true;
        System.DateTime mostRecentFeed = System.DateTime.MinValue;

        public PCDecoder(QueueThreadSafe _inQueue, QueueThreadSafe _outQueue) : base()
        {
            if (_inQueue == null)
            {
                throw new System.Exception("PCDecoder: inQueue is null");
            }
            if (_outQueue == null)
            {
                throw new System.Exception("PCDecoder: outQueue is null");
            }
            stats = new Stats(Name());
            try
            {
                inQueue = _inQueue;
                outQueue = _outQueue;
                decoder = cwipc.new_decoder();
                if (decoder == null)
                    throw new System.Exception("PCSUBReader: cwipc_new_decoder creation failed"); // Should not happen, should throw exception
                else
                {
                    Start();
                    Debug.Log($"{Name()} Inited");
                }

            }
            catch (System.Exception e)
            {
                Debug.Log($"{Name()}: Exception: {e.Message}");
                throw;
            }
            stats = new Stats(Name());
            debugColorize = Config.Instance.PCs.debugColorize;
        }

        public override string Name()
        {
            return $"{GetType().Name}#{instanceNumber}";
        }

        public override void Stop()
        {
            base.Stop();
            if (outQueue != null && !outQueue.IsClosed()) outQueue.Close();
        }

        public override void OnStop()
        {
            base.OnStop();
            lock (this)
            {
                decoder?.free();
                decoder = null;
                if (outQueue != null && !outQueue.IsClosed()) outQueue.Close();
            }
            if (debugThreading) Debug.Log($"{Name()} Stopped");
        }

        bool _FeedDecoder() {
            NativeMemoryChunk mc = (NativeMemoryChunk)inQueue.TryDequeue(0);
            if (mc == null) return false;
            mostRecentFeed = System.DateTime.Now;
            decoder.feed(mc.pointer, mc.length);
            mc.free();
            return true;
        }

        protected override void Update()
        {
            base.Update(); 
            lock (this)
            {
                // Feed data into the decoder, unless it already
                // has a pointcloud available, or a previously fed buffer hasn't been decoded yet.
                if (decoder == null) return;
                if (mostRecentFeed == System.DateTime.MinValue)
                {
                    if (!_FeedDecoder())
                    {
                        return;
                    }
                }
            }
            while (decoder.available(false))
            {
                // While the decoder has pointclouds available
                // push them into the output queue, and if there
                // are more input packets available feed the decoder
                // again.
                cwipc.pointcloud pc = decoder.get();
                Timedelta decodeDuration = (Timedelta)(System.DateTime.Now - mostRecentFeed).TotalMilliseconds;
                mostRecentFeed = System.DateTime.MinValue;
                if (pc == null)
                {
                    throw new System.Exception($"{Name()}: cwipc_decoder: available() true, but did not return a pointcloud");
                }
                if (debugColorize)
                {
                    int cnum = (instanceNumber % 6) + 1;
                    uint cmask = 0;
                    if ((cnum & 1) != 0) cmask |= 0x800000;
                    if ((cnum & 2) != 0) cmask |= 0x008000;
                    if ((cnum & 4) != 0) cmask |= 0x000080;
                    cwipc.pointcloud newpc = cwipc.colormap(pc, 0, cmask);
                    pc.free();
                    pc = newpc;
                }
                Timedelta queuedDuration = outQueue.QueuedDuration();
                bool dropped = !outQueue.Enqueue(pc);
                stats.statsUpdate(pc.count(), dropped, inQueue.QueuedDuration(), decodeDuration, queuedDuration);
                _FeedDecoder();
            }
        }

        protected class Stats : VRT.Core.BaseStats
        {
            public Stats(string name) : base(name) { }

            double statsTotalPoints = 0;
            double statsTotalPointclouds = 0;
            double statsTotalDropped = 0;
            double statsTotalInQueueDuration = 0;
            double statsTotalDecodeDuration = 0;
            double statsTotalQueuedDuration = 0;

            public void statsUpdate(int pointCount, bool dropped, Timedelta inQueueDuration, Timedelta decodeDuration, Timedelta queuedDuration)
            {
                statsTotalPoints += pointCount;
                statsTotalPointclouds++;
                statsTotalInQueueDuration += inQueueDuration;
                statsTotalDecodeDuration += decodeDuration;
                statsTotalQueuedDuration += queuedDuration;
                if (dropped) statsTotalDropped++;
                
                if (ShouldOutput())
                {
                    double factor = (statsTotalPointclouds == 0 ? 1 : statsTotalPointclouds);
                    Output($"fps={statsTotalPointclouds / Interval():F2}, fps_dropped={statsTotalDropped / Interval():F2}, points_per_cloud={(int)(statsTotalPoints / factor)}, decoder_queue_ms={(int)(statsTotalInQueueDuration / factor)}, decoder_ms={statsTotalDecodeDuration / factor:F2}, decoded_queue_ms={(int)(statsTotalQueuedDuration / factor)}");
                    Clear();
                    statsTotalPoints = 0;
                    statsTotalPointclouds = 0;
                    statsTotalDropped = 0;
                    statsTotalInQueueDuration = 0;
                    statsTotalQueuedDuration = 0;
                    statsTotalDecodeDuration = 0;
                }
            }
        }

        protected Stats stats;

    }
}