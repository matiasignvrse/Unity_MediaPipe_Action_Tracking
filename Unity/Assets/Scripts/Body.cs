using System;
using UnityEngine;

public class Body : MonoBehaviour
{
    public class AvatarTree
    {
        public Transform transf; // bone position
        public AvatarTree child; // child bone
        public AvatarTree parent; // parent bone
        public int idx;  // bone index
        public Quaternion quaternion; // initial roration

        public AvatarTree(Transform tf, int idx, Quaternion quaternion, AvatarTree parent = null)
        {
            this.transf = tf;
            this.parent = parent;
            this.idx = idx;
            this.quaternion = quaternion;
        }

        /// <summary>
        /// bone direction
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

    public Animator anim; // character controller
    public Transform hip; // 髋部
    public Transform spine; // 脊柱
    public Transform thorax; // 胸部
    public Transform neck; // 颈部
    public Transform head; // 头部
    public Transform nose; // 鼻子
    public Transform lHip; // 左髋
    public Transform lKnee; // 左膝
    public Transform lFoot; // 左脚
    public Transform rHip; // 右髋
    public Transform rKnee; // 右膝
    public Transform rFoot; // 右脚
    public Transform lSld; // 左肩
    public Transform lArm; // 左臂
    public Transform lEblow; // 左肘
    public Transform lWrist; // 左手腕
    public Transform rSld; // 右肩
    public Transform rArm; // 右臂
    public Transform rEblow; // 右肘
    public Transform rWrist; // 右手腕

    public AvatarTree Hip;
    public AvatarTree LHip;
    private AvatarTree LKnee;
    private AvatarTree LFoot;
    public AvatarTree RHip;
    private AvatarTree RKnee;
    private AvatarTree RFoot;
    private AvatarTree Spine;
    private AvatarTree Thorax;
    private AvatarTree Neck;
    private AvatarTree Head;
    private AvatarTree Nose;
    public AvatarTree LSld;
    private AvatarTree LEblow;
    private AvatarTree LWrist;
    public AvatarTree RSld;
    private AvatarTree REblow;
    private AvatarTree RWrist;
    private AvatarTree LArm;
    private AvatarTree RArm;

    public float[][] pose_data;  //body data
    public float lerp;

    private void Start()
    {
        InitAvatar();
        BulidTree();
    }

    private void InitAvatar()
    {
        if (anim != null)
        {
            lHip = anim.GetBoneTransform(HumanBodyBones.LeftUpperLeg);
            lKnee = anim.GetBoneTransform(HumanBodyBones.LeftLowerLeg);
            lFoot = anim.GetBoneTransform(HumanBodyBones.LeftFoot);
            rHip = anim.GetBoneTransform(HumanBodyBones.RightUpperLeg);
            rKnee = anim.GetBoneTransform(HumanBodyBones.RightLowerLeg);
            rFoot = anim.GetBoneTransform(HumanBodyBones.RightFoot);
            spine = anim.GetBoneTransform(HumanBodyBones.Spine);
            thorax = anim.GetBoneTransform(HumanBodyBones.Chest);
            neck = anim.GetBoneTransform(HumanBodyBones.Neck);
            head = anim.GetBoneTransform(HumanBodyBones.Head);
            lSld = anim.GetBoneTransform(HumanBodyBones.LeftShoulder);
            lArm = anim.GetBoneTransform(HumanBodyBones.LeftUpperArm);
            lEblow = anim.GetBoneTransform(HumanBodyBones.LeftLowerArm);
            lWrist = anim.GetBoneTransform(HumanBodyBones.LeftHand);
            rSld = anim.GetBoneTransform(HumanBodyBones.RightShoulder);
            rArm = anim.GetBoneTransform(HumanBodyBones.RightUpperArm);
            rEblow = anim.GetBoneTransform(HumanBodyBones.RightLowerArm);
            rWrist = anim.GetBoneTransform(HumanBodyBones.RightHand);
        }
    }

    private void BulidTree()
    {
        Hip = new AvatarTree(hip, -1, hip.rotation); // -1
        Spine = Hip.child = new AvatarTree(spine, -2, spine.rotation, Hip); // -2
        Thorax = Spine.child = new AvatarTree(thorax, -3, thorax.rotation, Spine); // -3
        Neck = Thorax.child = new AvatarTree(neck, -4, neck.rotation, Thorax); // -4
        Head = Neck.child = new AvatarTree(head, -5, head.rotation, Neck); // -5
        Nose = Head.child = new AvatarTree(nose, 0, nose.rotation, Head);
        LHip = new AvatarTree(lHip, 23, lHip.rotation);
        LKnee = LHip.child = new AvatarTree(lKnee, 25, lKnee.rotation, LHip);
        LFoot = LKnee.child = new AvatarTree(lFoot, 29, lFoot.rotation, LKnee);
        RHip = new AvatarTree(rHip, 24, rHip.rotation);
        RKnee = RHip.child = new AvatarTree(rKnee, 26, rHip.rotation, RHip);
        RFoot = RKnee.child = new AvatarTree(rFoot, 30, rFoot.rotation, RKnee);
        LSld = new AvatarTree(lSld, -6, lSld.rotation); // -6
        LArm = LSld.child = new AvatarTree(lArm, 11, lArm.rotation, LSld);
        LEblow = LArm.child = new AvatarTree(lEblow, 13, lEblow.rotation, LArm);
        LWrist = LEblow.child = new AvatarTree(lWrist, 15, lWrist.rotation, LEblow);
        RSld = new AvatarTree(rSld, -7, rSld.rotation); // -7
        RArm = RSld.child = new AvatarTree(rArm, 12, rArm.rotation, RSld);
        REblow = RArm.child = new AvatarTree(rEblow, 14, rEblow.rotation, RArm);
        RWrist = REblow.child = new AvatarTree(rWrist, 16, rWrist.rotation, REblow);
    }//bone tree

    // void Update()
    // {
    //     lerp += Time.deltaTime;
    //     if (lerp >= 1.0f)
    //     {
    //         lerp = 0;
    //     }
    //     if (pose_data != null)
    //     {
    //         UpdateTree(Hip, lerp);
    //         UpdateTree(RArm, lerp);
    //         UpdateTree(LArm, lerp);
    //         //UpdateTree(LHip, lerp);
    //         //UpdateTree(RHip, lerp);
    //     }
    // }
    void Update()
    {
        float rotationSpeed = 3.0f; // adjust rotation speed
        float t = rotationSpeed * Time.deltaTime;

        if (pose_data != null && pose_data.Length > 0)
        {
            UpdateTree(Hip, t);
            UpdateTree(RArm, t);
            UpdateTree(LArm, t);
            // UpdateTree(LHip, t);
            // UpdateTree(RHip, t);
        }
    }

    private Vector3 GetData(int idx)
    {
        float x, y, z;
        
        if (idx == -1)
        {
            x = (pose_data[23][0] + pose_data[24][0]) / 2;
            y = (pose_data[23][1] + pose_data[24][1]) / 2;
            z = (pose_data[23][2] + pose_data[24][2]) / 2;
        }
        else if (idx == -2)
        {
            x = ((pose_data[11][0] + pose_data[12][0]) / 2 + (pose_data[23][0] + pose_data[24][0]) / 2) / 2;
            y = ((pose_data[11][1] + pose_data[12][1]) / 2 + (pose_data[23][1] + pose_data[24][1]) / 2) / 2;
            z = ((pose_data[11][2] + pose_data[12][2]) / 2 + (pose_data[23][2] + pose_data[24][2]) / 2) / 2;
        }
        else if (idx == -3)
        {
            x = (pose_data[11][0] + pose_data[12][0]) / 2;
            y = (pose_data[11][1] + pose_data[12][1]) / 2;
            z = (pose_data[11][2] + pose_data[12][2]) / 2;
        }
        else if (idx == -4)
        {
            x = ((pose_data[8][0] + pose_data[7][0]) / 2 + (pose_data[11][0] + pose_data[12][0]) / 2) / 2;
            y = ((pose_data[8][1] + pose_data[7][1]) / 2 + (pose_data[11][1] + pose_data[12][1]) / 2) / 2;
            z = ((pose_data[8][2] + pose_data[7][2]) / 2 + (pose_data[11][2] + pose_data[12][2]) / 2) / 2;
        }
        else if (idx == -5)
        {
            x = (pose_data[8][0] + pose_data[7][0]) / 2;
            y = (pose_data[8][1] + pose_data[7][1]) / 2;
            z = (pose_data[8][2] + pose_data[7][2]) / 2;
        }
        else if (idx == -6)
        {
            x = (pose_data[11][0] * 3 + pose_data[12][0]) / 4;
            y = (pose_data[11][1] * 3 + pose_data[12][1]) / 4;
            z = (pose_data[11][2] * 3 + pose_data[12][2]) / 4;
        }
        else if (idx == -7)
        {
            x = (pose_data[11][0] + pose_data[12][0] * 3) / 4;
            y = (pose_data[11][1] + pose_data[12][1] * 3) / 4;
            z = (pose_data[11][2] + pose_data[12][2] * 3) / 4;
        }
        else
        {
            if (idx >= 0 && idx < pose_data.Length)
            {
                x = pose_data[idx][0];
                y = pose_data[idx][1];
                z = pose_data[idx][2];
            }
            else
            {
                Debug.LogWarning($"Index {idx} is out of bounds for pose_data array.");
                x = y = z = 0;
            }
        }
        return new Vector3(-x, y, -z);
        //return new Vector3(-x, y, z);
        //return new Vector3(x, y, -z);
        //return new Vector3(x, y, z);

    }

    private void UpdateTree(AvatarTree tree, float lerp)
    {
        if (tree.parent != null)
        {
            UpdateBone(tree, lerp);
        }
        if (tree.child != null)
        {
            UpdateTree(tree.child, lerp);
        }
    }

    private void UpdateBone(AvatarTree tree, float lerp)
    {
        var dir1 = tree.GetDir();
        var dir2 = GetData(tree.parent.idx) - GetData(tree.idx);
        Quaternion rot = Quaternion.FromToRotation(dir1, dir2);
        Quaternion rot1 = tree.parent.transf.rotation;
        tree.parent.transf.rotation = Quaternion.Lerp(rot1, rot * rot1, lerp);
    }//bone rotation
}



