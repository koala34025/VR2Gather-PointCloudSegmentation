﻿using System.Collections.Generic;
using LitJson;
using System.Reflection;

namespace OrchestratorWrapping
{
    public static class Helper
    {
        // Parse JsonData and returns the appropriate element
        public static List<T> ParseElementsList<T>(JsonData dataList) where T : OrchestratorElement
        {
            List<T> list = new List<T>();
            for (int i = 0; i < dataList.Count; i++)
            {
                object[] arg = { dataList[i] };
                // call the class function that knows how to parse the Json Data
                T element = (T)(typeof(T).InvokeMember("ParseJsonData", BindingFlags.InvokeMethod, null, null, arg));
                list.Add(element);
            }
            return list;
        }
    }
}
