using System;
using System.Collections.Generic;
using UnityEngine;

public class Hand : MonoBehaviour
{
    public class AvatarTree
    {
        public Transform transf; 
        public AvatarTree[] childs; 
        public AvatarTree parent; 
        public int idx;  
        public Quaternion quaternion; 

        public AvatarTree(Transform tf, int count, int idx, Quaternion quaternion, AvatarTree parent = null)
        {
            this.transf = tf;
            this.parent = parent;
            this.idx = idx;
            this.quaternion = quaternion;
            if (count > 0)
            {
                childs = new AvatarTree[count];
            }
        }

        /// <summary>
        /// </summary>
        /// <returns></returns>
        public Vector3 GetDir()
        {
            if (parent != null)
            {
                return transf.position - parent.transf.position;
            }
            return Vector3.up;
        }
    }

    public Transform l_wrist;
    public Transform l_thumb1;
    public Transform l_thumb2;
    public Transform l_thumb3;
    public Transform l_thumb4;
    public Transform l_index1;
    public Transform l_index2;
    public Transform l_index3;
    public Transform l_index4;
    public Transform l_middle1;
    public Transform l_middle2;
    public Transform l_middle3;
    public Transform l_middle4;
    public Transform l_ring1;
    public Transform l_ring2;
    public Transform l_ring3;
    public Transform l_ring4;
    public Transform l_pinky1;
    public Transform l_pinky2;
    public Transform l_pinky3;
    public Transform l_pinky4;

    public Transform r_wrist;
    public Transform r_thumb1;
    public Transform r_thumb2;
    public Transform r_thumb3;
    public Transform r_thumb4;
    public Transform r_index1;
    public Transform r_index2;
    public Transform r_index3;
    public Transform r_index4;
    public Transform r_middle1;
    public Transform r_middle2;
    public Transform r_middle3;
    public Transform r_middle4;
    public Transform r_ring1;
    public Transform r_ring2;
    public Transform r_ring3;
    public Transform r_ring4;
    public Transform r_pinky1;
    public Transform r_pinky2;
    public Transform r_pinky3;
    public Transform r_pinky4;

    // bone tree
    public AvatarTree L_Wrist;
    private AvatarTree L_Thumb1;
    private AvatarTree L_Thumb2;
    private AvatarTree L_Thumb3;
    private AvatarTree L_Thumb4;
    private AvatarTree L_Index1;
    private AvatarTree L_Index2;
    private AvatarTree L_Index3;
    private AvatarTree L_Index4;
    private AvatarTree L_Middle1;
    private AvatarTree L_Middle2;
    private AvatarTree L_Middle3;
    private AvatarTree L_Middle4;
    private AvatarTree L_Ring1;
    private AvatarTree L_Ring2;
    private AvatarTree L_Ring3;
    private AvatarTree L_Ring4;
    private AvatarTree L_Pinky1;
    private AvatarTree L_Pinky2;
    private AvatarTree L_Pinky3;
    private AvatarTree L_Pinky4;

    public AvatarTree R_Wrist;
    private AvatarTree R_Thumb1;
    private AvatarTree R_Thumb2;
    private AvatarTree R_Thumb3;
    private AvatarTree R_Thumb4;
    private AvatarTree R_Index1;
    private AvatarTree R_Index2;
    private AvatarTree R_Index3;
    private AvatarTree R_Index4;
    private AvatarTree R_Middle1;
    private AvatarTree R_Middle2;
    private AvatarTree R_Middle3;
    private AvatarTree R_Middle4;
    private AvatarTree R_Ring1;
    private AvatarTree R_Ring2;
    private AvatarTree R_Ring3;
    private AvatarTree R_Ring4;
    private AvatarTree R_Pinky1;
    private AvatarTree R_Pinky2;
    private AvatarTree R_Pinky3;
    private AvatarTree R_Pinky4;

    public float[][] left_hand_data;  
    public float[][] right_hand_data; 
    public float lerp;

    private void Start()
    {
        BulidTree();
    }

    private void BulidTree()
    {
        // left hand bone tree
        L_Wrist = new AvatarTree(l_wrist, 5, 0, l_wrist.rotation); // left wrist
        L_Thumb1 = L_Wrist.childs[0] = new AvatarTree(l_thumb1, 1, 1, l_thumb1.rotation, L_Wrist);
        L_Index1 = L_Wrist.childs[1] = new AvatarTree(l_index1, 1, 5, l_index1.rotation, L_Wrist);
        L_Middle1 = L_Wrist.childs[2] = new AvatarTree(l_middle1, 1, 9, l_middle1.rotation, L_Wrist);
        L_Ring1 = L_Wrist.childs[3] = new AvatarTree(l_ring1, 1, 13, l_ring1.rotation, L_Wrist);
        L_Pinky1 = L_Wrist.childs[4] = new AvatarTree(l_pinky1, 1, 17, l_pinky1.rotation, L_Wrist);

        L_Thumb2 = L_Thumb1.childs[0] = new AvatarTree(l_thumb2, 1, 2, l_thumb2.rotation, L_Thumb1);
        L_Thumb3 = L_Thumb2.childs[0] = new AvatarTree(l_thumb3, 1, 3, l_thumb3.rotation, L_Thumb2);
        L_Thumb4 = L_Thumb3.childs[0] = new AvatarTree(l_thumb4, 0, 4, l_thumb4.rotation, L_Thumb3);

        L_Index2 = L_Index1.childs[0] = new AvatarTree(l_index2, 1, 6, l_index2.rotation, L_Index1);
        L_Index3 = L_Index2.childs[0] = new AvatarTree(l_index3, 1, 7, l_index3.rotation, L_Index2);
        L_Index4 = L_Index3.childs[0] = new AvatarTree(l_index4, 0, 8, l_index4.rotation, L_Index3);

        L_Middle2 = L_Middle1.childs[0] = new AvatarTree(l_middle2, 1, 10, l_middle2.rotation, L_Middle1);
        L_Middle3 = L_Middle2.childs[0] = new AvatarTree(l_middle3, 1, 11, l_middle3.rotation, L_Middle2);
        L_Middle4 = L_Middle3.childs[0] = new AvatarTree(l_middle4, 0, 12, l_middle4.rotation, L_Middle3);

        L_Ring2 = L_Ring1.childs[0] = new AvatarTree(l_ring2, 1, 14, l_ring2.rotation, L_Ring1);
        L_Ring3 = L_Ring2.childs[0] = new AvatarTree(l_ring3, 1, 15, l_ring3.rotation, L_Ring2);
        L_Ring4 = L_Ring3.childs[0] = new AvatarTree(l_ring4, 0, 16, l_ring4.rotation, L_Ring3);

        L_Pinky2 = L_Pinky1.childs[0] = new AvatarTree(l_pinky2, 1, 18, l_pinky2.rotation, L_Pinky1);
        L_Pinky3 = L_Pinky2.childs[0] = new AvatarTree(l_pinky3, 1, 19, l_pinky3.rotation, L_Pinky2);
        L_Pinky4 = L_Pinky3.childs[0] = new AvatarTree(l_pinky4, 0, 20, l_pinky4.rotation, L_Pinky3);

        // right hand bone tree
        R_Wrist = new AvatarTree(r_wrist, 5, 0, r_wrist.rotation); // right wrist
        R_Thumb1 = R_Wrist.childs[0] = new AvatarTree(r_thumb1, 1, 1, r_thumb1.rotation, R_Wrist);
        R_Index1 = R_Wrist.childs[1] = new AvatarTree(r_index1, 1, 5, r_index1.rotation, R_Wrist);
        R_Middle1 = R_Wrist.childs[2] = new AvatarTree(r_middle1, 1, 9, r_middle1.rotation, R_Wrist);
        R_Ring1 = R_Wrist.childs[3] = new AvatarTree(r_ring1, 1, 13, r_ring1.rotation, R_Wrist);
        R_Pinky1 = R_Wrist.childs[4] = new AvatarTree(r_pinky1, 1, 17, r_pinky1.rotation, R_Wrist);

        R_Thumb2 = R_Thumb1.childs[0] = new AvatarTree(r_thumb2, 1, 2, r_thumb2.rotation, R_Thumb1);
        R_Thumb3 = R_Thumb2.childs[0] = new AvatarTree(r_thumb3, 1, 3, r_thumb3.rotation, R_Thumb2);
        R_Thumb4 = R_Thumb3.childs[0] = new AvatarTree(r_thumb4, 0, 4, r_thumb4.rotation, R_Thumb3);

        R_Index2 = R_Index1.childs[0] = new AvatarTree(r_index2, 1, 6, r_index2.rotation, R_Index1);
        R_Index3 = R_Index2.childs[0] = new AvatarTree(r_index3, 1, 7, r_index3.rotation, R_Index2);
        R_Index4 = R_Index3.childs[0] = new AvatarTree(r_index4, 0, 8, r_index4.rotation, R_Index3);

        R_Middle2 = R_Middle1.childs[0] = new AvatarTree(r_middle2, 1, 10, r_middle2.rotation, R_Middle1);
        R_Middle3 = R_Middle2.childs[0] = new AvatarTree(r_middle3, 1, 11, r_middle3.rotation, R_Middle2);
        R_Middle4 = R_Middle3.childs[0] = new AvatarTree(r_middle4, 0, 12, r_middle4.rotation, R_Middle3);

        R_Ring2 = R_Ring1.childs[0] = new AvatarTree(r_ring2, 1, 14, r_ring2.rotation, R_Ring1);
        R_Ring3 = R_Ring2.childs[0] = new AvatarTree(r_ring3, 1, 15, r_ring3.rotation, R_Ring2);
        R_Ring4 = R_Ring3.childs[0] = new AvatarTree(r_ring4, 0, 16, r_ring4.rotation, R_Ring3);

        R_Pinky2 = R_Pinky1.childs[0] = new AvatarTree(r_pinky2, 1, 18, r_pinky2.rotation, R_Pinky1);
        R_Pinky3 = R_Pinky2.childs[0] = new AvatarTree(r_pinky3, 1, 19, r_pinky3.rotation, R_Pinky2);
        R_Pinky4 = R_Pinky3.childs[0] = new AvatarTree(r_pinky4, 0, 20, r_pinky4.rotation, R_Pinky3);
    }

    void Update()
    {
        lerp += Time.deltaTime;
        if (lerp >= 1.0f)
        {
            lerp = 0;
        }

        bool hasLeftData = left_hand_data != null && left_hand_data.Length > 0;
        bool hasRightData = right_hand_data != null && right_hand_data.Length > 0;

        if (hasLeftData)
        {
            UpdateTree(L_Wrist, lerp, true);
        }
        if (hasRightData)
        {
            UpdateTree(R_Wrist, lerp, false);
        }
    }

    private Vector3 GetData(int idx, bool isLeft)
    {
        float x, y, z;
        float[] dataArray = isLeft ? left_hand_data[idx] : right_hand_data[idx];

        // index check
        if (idx >= 0 && idx < (isLeft ? left_hand_data.Length : right_hand_data.Length))
        {
            x = dataArray[0];
            y = dataArray[1];
            z = dataArray[2];
        }
        else
        {
            Debug.LogWarning($"Index {idx} is out of bounds for {(isLeft ? "left_hand_data" : "right_hand_data")} array.");
            x = y = z = 0;
        }

        return new Vector3(-x, y, -z);
    }

    private void UpdateTree(AvatarTree tree, float lerp, bool isLeft)
    {
        if (tree.parent != null)
        {
            UpdateBone(tree, lerp, isLeft);
        }
        if (tree.childs != null)
        {
            foreach (var child in tree.childs)
            {
                if (child != null)
                {
                    UpdateTree(child, lerp, isLeft);
                }
            }
        }
    }

    private void UpdateBone(AvatarTree tree, float lerp, bool isLeft)
    {
        Vector3 dir1 = tree.GetDir();
        Vector3 parentPos = GetData(tree.parent.idx, isLeft);
        Vector3 childPos = GetData(tree.idx, isLeft);
        Vector3 dir2 = parentPos - childPos;

        Quaternion rot = Quaternion.FromToRotation(dir1, dir2);
        Quaternion rot1 = tree.parent.transf.rotation;
        tree.parent.transf.rotation = Quaternion.Lerp(rot1, rot * rot1, lerp);
    }
}
