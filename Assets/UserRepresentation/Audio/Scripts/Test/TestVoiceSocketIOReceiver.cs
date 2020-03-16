﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestVoiceSocketIOReceiver : MonoBehaviour
{
    Workers.BaseWorker reader;
    Workers.BaseWorker codec;
    Workers.AudioPreparer preparer;

    Workers.Token token;

    public SocketIOConnection socketIOConnection;
    public int userID;

    AudioSource audioSource;
    // Start is called before the first frame update
    IEnumerator Start() {
        yield return socketIOConnection.WaitConnection();

        audioSource = gameObject.AddComponent<AudioSource>();
//        audioSource.clip = AudioClip.Create("clip0", 320, 1, 16000, true, OnAudioRead);
        audioSource.clip = AudioClip.Create("clip0", 320, 1, 16000, false);
        audioSource.loop = true;
        audioSource.spatialBlend = 1.0f;
        audioSource.Play();

        reader = new Workers.SocketIOReader(socketIOConnection, userID.ToString());
        codec = new Workers.VoiceDecoder();
        preparer = new Workers.AudioPreparer();
        reader.AddNext(codec).AddNext(preparer).AddNext(reader);
        reader.token = token = new Workers.Token();
    }

    void OnDestroy()
    {
        reader?.Stop();
        codec?.Stop();
        preparer?.Stop();
    }

    // Buffer is filled 2.5 times per second (every 400ms). 
    void OnAudioRead(float[] data) {
        if (preparer == null || !preparer.GetAudioBuffer(data, data.Length))
            System.Array.Clear(data, 0, data.Length);
    }

    float[] tmpBuffer;
    void OnAudioFilterRead(float[] data, int channels) {
        if (tmpBuffer == null) tmpBuffer = new float[data.Length];
        if (preparer != null && preparer.GetAudioBuffer(tmpBuffer, tmpBuffer.Length)) {
            int cnt = 0;
            do { data[cnt] += tmpBuffer[cnt]; } while (++cnt < data.Length);
        }
    }
}