﻿using System.Collections;
using System.Collections.Generic;
using System.IO;
using System;
using UnityEngine;
using VRT.Transport.Dash;
using VRT.Orchestrator.Wrapping;
using VRT.Core;

namespace VRT.Transport.SocketIO
{
    public class SocketIOReader : BaseReader, ISocketReader
    {
        PCSubReader.TileDescriptor[] descriptors;

        User user;

        public SocketIOReader(User user, string remoteStream, PCSubReader.TileDescriptor[] descriptors) : base(WorkerType.End)
        {
            this.user = user;
            if (descriptors == null)
            {
                throw new System.Exception($"{Name()}: descriptors is null");
            }
            this.descriptors = descriptors;
            try
            {
                for (int i = 0; i < this.descriptors.Length; ++i)
                {
                    this.descriptors[i].name = $"{user.userId}{remoteStream}#{i}";
                    Debug.Log($"[FPA] RegisterForDataStream userId {user.userId} StreamType {this.descriptors[i].name}");
                    OrchestratorWrapper.instance.RegisterForDataStream(user.userId, this.descriptors[i].name);
                }
                OrchestratorWrapper.instance.OnDataStreamReceived += OnDataPacketReceived;
                stats = new Stats(Name());
                Start();
                Debug.Log($"{Name()}: Started {remoteStream}.");
            }
            catch (System.Exception e)
            {
                Debug.Log($"{Name()}: Exception: {e.Message}");
                throw;
            }
        }

        public SocketIOReader(User user, string remoteStream, QueueThreadSafe outQueue)
        : this(user,
            remoteStream,
              new PCSubReader.TileDescriptor[]
              {
                  new PCSubReader.TileDescriptor()
                  {
                      outQueue = outQueue
                  }
              }
            )
        {
            stats = new Stats(Name());
        }

        static int instanceCounter = 0;
        int instanceNumber = instanceCounter++;
        public override string Name()
        {
            return $"{GetType().Name}#{instanceNumber}";
        }

        public override void Stop()
        {
            base.Stop();
            for (int i = 0; i < descriptors.Length; ++i)
            {
                descriptors[i].outQueue?.Close();
                Debug.Log($"[FPA] {Name()}: Stopped.");
                if (OrchestratorWrapper.instance != null && OrchestratorController.Instance.SelfUser != null)
                    OrchestratorWrapper.instance.UnregisterFromDataStream(OrchestratorController.Instance.SelfUser.userId, descriptors[i].name);
            }
        }
        private void OnDataPacketReceived(UserDataStreamPacket pPacket)
        {
            int id = 0;
            string strID = pPacket.dataStreamType.Substring(pPacket.dataStreamType.LastIndexOf('#') + 1);
            if (int.TryParse(strID, out id))
            {
                if (pPacket.dataStreamType == descriptors[id].name)
                {
                    byte[] hdr_timestamp = new byte[sizeof(long)];
                    Array.Copy(pPacket.dataStreamPacket, hdr_timestamp, sizeof(long));
                    long timestamp = BitConverter.ToInt64(hdr_timestamp, 0);
                    BaseMemoryChunk chunk = new NativeMemoryChunk(pPacket.dataStreamPacket.Length-sizeof(long));
                    chunk.info.timestamp = timestamp;
                    System.Runtime.InteropServices.Marshal.Copy(pPacket.dataStreamPacket, sizeof(long), chunk.pointer, chunk.length);
                    // xxxjack note: this means we are _not_ distinghuising tiles for socketIO. Should be fixed, but difficult.
                    bool didDrop = true;
                    if (id < descriptors.Length)
                    {
                        didDrop = !descriptors[id].outQueue.Enqueue(chunk);
                    } else
                    {
                        Debug.LogWarning($"Name(): drop packet for unknown stream {id}");
                    }
                    stats.statsUpdate(chunk.length, didDrop, timestamp, id);

                }
            }
            else
            {
                Debug.Log($"[FPA] ERROR parsing {strID}.");
            }
        }

        public void OnData(byte[] data)
        {
        }

        protected override void Update()
        {
            base.Update();
        }

        protected class Stats : VRT.Core.BaseStats
        {
            public Stats(string name) : base(name) { }

            double statsTotalBytes = 0;
            double statsTotalPackets = 0;
            double statsTotalDrops = 0;
            
            public void statsUpdate(int nBytes, bool dropped, long timestamp, int streamId)
            {
                statsTotalBytes += nBytes;
                statsTotalPackets++;
                if (dropped) statsTotalDrops++;
                if (ShouldOutput())
                {
                    Output($"fps={statsTotalPackets / Interval():F2}, dropped_fps={statsTotalDrops / Interval():F2}, bytes_per_packet={(int)(statsTotalBytes / statsTotalPackets)}, last_stream_index={streamId}, last_timestamp={timestamp}");
                }
                if (ShouldClear())
                {
                    Clear();
                    statsTotalBytes = 0;
                    statsTotalPackets = 0;
                    statsTotalDrops = 0;
                }
            }
        }

        protected Stats stats;

    }

}
