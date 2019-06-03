﻿using System;
using System.Runtime.InteropServices;
using UnityEngine;

public class PCCompressedDirectoryReader : PCBaseReader {
    string[] allFilenames;
    int      currentFile;
    IntPtr  decoder;

    public PCCompressedDirectoryReader(string dirname) {
        currentFile = 0;
        allFilenames = System.IO.Directory.GetFiles(Application.streamingAssetsPath + "/" + dirname);
        System.IntPtr errorPtr = System.IntPtr.Zero;

        decoder = API_cwipc_codec.cwipc_new_decoder(ref errorPtr);
        if (decoder == IntPtr.Zero)
        {
            string errorMessage = Marshal.PtrToStringAuto(errorPtr);
            Debug.LogError("PCCompressedDirectoryReader: cwipc_new_decoder: " + errorMessage);
        }
    }

    public void free() {
    }

    public bool eof() {
        return allFilenames.Length == 0;
    }

    public bool available(bool wait) {
        return allFilenames.Length != 0;
    }

    public PointCloudFrame get()
    {
        if (decoder == IntPtr.Zero) {
            Debug.LogError("PCCompressedDirectoryReader: cwipc_decoder: no decoder available");
            return null;
        }

        string filename = allFilenames[currentFile];
        currentFile = (currentFile + 1) % allFilenames.Length;

        var bytes = System.IO.File.ReadAllBytes(filename);
        var ptr = Marshal.UnsafeAddrOfPinnedArrayElement(bytes, 0);
        API_cwipc_codec.cwipc_decoder_feed(decoder, ptr, bytes.Length);
        bool ok = API_cwipc_util.cwipc_source_available(decoder, true);
        if (!ok) {
            Debug.LogError("PCCompressedDirectoryReader: cwipc_decoder: no pointcloud available");
            return null;
        }
        var pc = API_cwipc_util.cwipc_source_get(decoder);
        if (pc == null) {
            Debug.LogError("PCCompressedDirectoryReader: cwipc_decoder: did not return a pointcloud");
            return null;
        }

        return new PointCloudFrame(pc);
    }
}