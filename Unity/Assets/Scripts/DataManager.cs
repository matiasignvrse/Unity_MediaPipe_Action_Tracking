// using System.Net;
// using System.Text;
// using System.Net.Sockets;
// using System.Threading;
// using UnityEngine;
// using System;

// /// <summary>
// /// ���ݹ��������ջ�Python�˵�����
// /// </summary>
// public class DataManager : MonoBehaviour
// {
//     /// <summary>
//     /// 
//     /// </summary>
//     public Body body;
//     public Hand hand;
//     Thread receiveThread;
//     UdpClient client;
//     public int port = 5054;
//     public string[] data;

//     void Start()
//     {
//         //����Socket�ջ�����
//         receiveThread = new Thread(new ThreadStart(ReceiveData));
//         receiveThread.IsBackground = true;
//         receiveThread.Start();
//     }

//     private void ReceiveData()
//     {
//         client = new UdpClient(port);
//         while (true)
//         {
//             try
//             {
//                 //�����������ݸ�������ֲ�
//                 IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, 0);
//                 byte[] dataByte = client.Receive(ref anyIP);
//                 data = Encoding.UTF8.GetString(dataByte).Split(';');
//                 hand.data = data[0];
//                 body.data = data[1];
//             }
//             catch (Exception e)
//             {
//                 Debug.Log(e);
//             }
//         }
//     }//��������

//     void Update()
//     {

//     }
// }

using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
using Newtonsoft.Json.Linq;
using System.Collections.Generic;
using System.Linq;

// 定义 Landmark 类来匹配 JSON 中的关键点结构
[Serializable]
public class Landmark
{
    public float x;
    public float y;
    public float z;
}

// 定义 Data 类来匹配整个 JSON 结构
[Serializable]
public class Data
{
    public List<Landmark> left_hand;
    public List<Landmark> right_hand;
    public List<Landmark> pose;
}

/// <summary>
/// 数据管理器，接收Python发送的数据
/// </summary>
public class DataManager : MonoBehaviour
{
    public Body body;
    public Hand hand;
    Thread receiveThread;
    UdpClient client;
    public int port = 5054;
    public string receivedData;

    void Start()
    {
        // 启动接收线程
        receiveThread = new Thread(new ThreadStart(ReceiveData));
        receiveThread.IsBackground = true;
        receiveThread.Start();
    }

    private void ReceiveData()
    {
        client = new UdpClient(port);
        while (true)
        {
            try
            {
                // 接收数据
                IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, 0);
                byte[] dataByte = client.Receive(ref anyIP);
                receivedData = Encoding.UTF8.GetString(dataByte);

                // 打印接收到的数据
                Debug.Log($"Received data: {receivedData}");

                // 移除 <EOM> 结尾
                if (receivedData.EndsWith("<EOM>"))
                {
                    receivedData = receivedData.Substring(0, receivedData.Length - "<EOM>".Length);
                    Debug.Log("Removed <EOM> from received data.");
                }

                // 解析 JSON 数据
                Data jsonData = JObject.Parse(receivedData).ToObject<Data>();

                // 解析手部数据并转换为 float[][]
                if (jsonData.left_hand != null && jsonData.left_hand.Count > 0)
                {
                    hand.left_hand_data = jsonData.left_hand.Select(l => new float[] { l.x, l.y, l.z }).ToArray();
                }
                else
                {
                    hand.left_hand_data = null;
                }

                if (jsonData.right_hand != null && jsonData.right_hand.Count > 0)
                {
                    hand.right_hand_data = jsonData.right_hand.Select(l => new float[] { l.x, l.y, l.z }).ToArray();
                }
                else
                {
                    hand.right_hand_data = null;
                }

                // 解析身体数据并转换为 float[][]
                if (jsonData.pose != null && jsonData.pose.Count > 0)
                {
                    body.pose_data = jsonData.pose.Select(l => new float[] { l.x, l.y, l.z }).ToArray();
                }
                else
                {
                    body.pose_data = null;
                }
            }
            catch (Exception e)
            {
                Debug.LogError($"Error receiving or parsing data: {e.Message}\nReceived Data: {receivedData}");
            }
        }
    }

    void Update()
    {
        // 可以在这里添加一些 UI 更新或其他逻辑
    }
}
