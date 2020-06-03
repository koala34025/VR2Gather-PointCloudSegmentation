﻿using UnityEngine;

namespace Workers {
    public class MeshPreparer : BaseWorker {
        bool isReady = false;
        Unity.Collections.NativeArray<PointCouldVertex> vertexArray;
        System.IntPtr currentBuffer;
        int PointCouldVertexSize;
        Vector3[] points;
        int[] indices;
        Color32[] colors;
        float currentCellSize = 0.008f;
        QueueThreadSafe InQueue;

        public MeshPreparer(QueueThreadSafe _InQueue) : base(WorkerType.End) {
            if (_InQueue == null)
            {
                throw new System.Exception("MeshPreparer: InQueue is null");
            }
            InQueue = _InQueue;
            PointCouldVertexSize = System.Runtime.InteropServices.Marshal.SizeOf(typeof(PointCouldVertex));
            Start();
        }

        public override void OnStop() {
            base.OnStop();
            if (vertexArray.Length != 0) vertexArray.Dispose();
        }

        [System.Runtime.InteropServices.StructLayout(System.Runtime.InteropServices.LayoutKind.Sequential)] // Also tried with Pack=1
        public struct PointCouldVertex {
            public Vector3 vertex;
            public Color32 color;
        }

        protected override void Update() {
            base.Update();
            if (InQueue._CanDequeue() && !isReady) {
                cwipc.pointcloud pc = (cwipc.pointcloud)InQueue.Dequeue();
                unsafe {
                    int bufferSize = pc.get_uncompressed_size();
                    currentCellSize = pc.cellsize();

                    // xxxjack if currentCellsize is != 0 it is the size at which the points should be displayed
                    int size = bufferSize / PointCouldVertexSize;
                    int dampedSize = (int)(size * Config.Instance.memoryDamping);
                    if (vertexArray.Length < dampedSize) {
                        vertexArray = new Unity.Collections.NativeArray<PointCouldVertex>(dampedSize, Unity.Collections.Allocator.Persistent);
                        currentBuffer = (System.IntPtr)Unity.Collections.LowLevel.Unsafe.NativeArrayUnsafeUtility.GetUnsafePtr(vertexArray);
                    }
                    int ret = pc.copy_uncompressed(currentBuffer, bufferSize);
                    pc.free();
                    // Check that sizes make sense. Note that copy_uncompressed returns the number of points
                    if (ret != size) {
                        Debug.LogError($"MeshPreparer: decoding problem: copy_uncompressed() size={ret}, get_uncompressed_size()={bufferSize}, vertexSize={size}");
                    }

                    points = new Vector3[size];
                    indices = new int[size];
                    colors = new Color32[size];
                    for (int i = 0; i < size; i++) {
                        points[i] = vertexArray[i].vertex;
                        indices[i] = i;
                        colors[i] = vertexArray[i].color;
                    }
                    lock (this) isReady = true;
                    //Next();
                }
            }
        }

        public bool GetMesh(ref Mesh mesh) {
            lock (this) {
                if (isReady) {
                    mesh.Clear();
                    mesh.vertices = points;
                    mesh.colors32 = colors;
                    // mesh.SetIndices(indices, 0, indices.Length, MeshTopology.Points, 0);
                    mesh.SetIndices(indices, MeshTopology.Points, 0);
                    isReady = false;
                    return true;
                }
            }
            return false;
        }

        public float GetPointSize() {
            if (currentCellSize > 0.0000f) return currentCellSize;
            else return 0.008f;
        }
    }
}
