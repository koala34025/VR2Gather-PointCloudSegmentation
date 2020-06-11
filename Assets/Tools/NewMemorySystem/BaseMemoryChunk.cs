﻿using System;
using System.Collections.Generic;
using UnityEngine;

public class BaseMemoryChunkReferences {
    static List<Type> types = new List<Type>();
    public static void AddReference(Type _type) {
        lock(types) {
            types.Add(_type);
        }
    }
    public static void DeleteReference(Type _type) {
        lock (types) {
            types.Remove(_type);
        }
    }

    public static void ShowTotalRefCount() {
        lock (types) {
            if (types.Count == 0) return;
            Debug.Log($"BaseMemoryChunkReferences: {types.Count} TotalRefCount pending:");
            for (int i = 0; i < types.Count; ++i)
                Debug.Log($"BaseMemoryChunkReferences: [{i}] --> {types[i]}");
        }
    }
}

public class BaseMemoryChunk {
    protected IntPtr        _pointer;
    int                     refCount;
    public sub.FrameInfo    info;
    public int              length { get; protected set; }

    protected BaseMemoryChunk(IntPtr _pointer) {
        if (_pointer== IntPtr.Zero)  throw new Exception("BaseMemoryChunk: constructor called with null pointer");
        this._pointer = _pointer;
        refCount = 1;
        BaseMemoryChunkReferences.AddReference( this.GetType() );
    }

    protected BaseMemoryChunk() {
        refCount = 1;
        BaseMemoryChunkReferences.AddReference(this.GetType());
    }

    public BaseMemoryChunk AddRef() {
        lock (this)
        {
            refCount++;
            return this;
        }
    }
    public IntPtr pointer { get { return _pointer; } }

    public void free() {
        lock (this)
        {
            if ( --refCount < 1) {
                if (refCount < 0)
                {
                    throw new System.Exception($"BaseMemoryChunk.free: refCount={refCount}");
                }
                if (_pointer!=IntPtr.Zero) {
                    onfree();
                    _pointer = IntPtr.Zero;
                    BaseMemoryChunkReferences.DeleteReference(this.GetType());
                }
            }
        }
    }

    protected virtual void onfree() {
    }
}