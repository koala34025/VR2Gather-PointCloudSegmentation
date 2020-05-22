using UnityEngine;

namespace Workers {
	public class AudioSubReader : BaseSubReader {
        public AudioSubReader(string _url, string _streamName, int _streamNumber, int _initialDelay, QueueThreadSafe _outQueue, bool _bDropFrames = false)
        : base(_url, _streamName, _initialDelay, _bDropFrames)
        {
            outQueues = new QueueThreadSafe[1] { _outQueue };
            streamIndexes = new int[1] { _streamNumber };
        }
    }
}