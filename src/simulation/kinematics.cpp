#include "simulation/kinematics.h"

#include "Eigen/Dense"

#include "acclaim/bone.h"
#include "util/helper.h"

namespace kinematics {
void forwardSolver(const acclaim::Posture& posture, acclaim::Bone* bone) {
    while (bone != NULL) {
        bone->rotation = bone->parent ? 
            Eigen::Affine3d(bone->parent->rotation * bone->rot_parent_current * util::rotateDegreeZYX(posture.bone_rotations[bone->idx])) : 
            Eigen::Affine3d(bone->rot_parent_current * util::rotateDegreeZYX(posture.bone_rotations[bone->idx]));

        bone->start_position = bone->parent ? bone->parent->end_position + posture.bone_translations[bone->idx] : posture.bone_translations[bone->idx];
        bone->end_position = bone->start_position + bone->rotation * bone->dir * bone->length;
        
        if (bone->child != NULL) {
            forwardSolver(posture, bone->child);
        }
        
        bone = bone->sibling;
    }
}

#define PI 3.1415926
std::vector<acclaim::Posture> timeWarper(const std::vector<acclaim::Posture>& postures, int keyframe_old,
                                         int keyframe_new) {
    int total_frames = static_cast<int>(postures.size());
    int total_bones = static_cast<int>(postures[0].bone_rotations.size());
    std::vector<acclaim::Posture> new_postures = postures;

    for (int i = 0; i < total_frames; ++i) {
        for (int j = 0; j < total_bones; ++j) {
            float weights = float(abs(i - keyframe_new)) / total_frames;
            int target_frame = i * keyframe_old / keyframe_new;

            target_frame = target_frame > total_frames - 1 ? total_frames - 1 : target_frame;
            
            new_postures[i].bone_translations[j] = weights * postures[i].bone_translations[j] + (1 - weights) * postures[target_frame].bone_translations[j];
            
            Eigen::Quaternion temp_q = util::rotateDegreeXYZ(postures[i].bone_rotations[j])
                                            .slerp(1.0f - weights, util::rotateDegreeXYZ(postures[target_frame].bone_rotations[j]));
            Eigen::Vector3d temp_v = temp_q.toRotationMatrix().eulerAngles(0, 1, 2) * (180.0f / PI);
            
            new_postures[i].bone_rotations[j] = Eigen::Vector4d(temp_v[0], temp_v[1], temp_v[2], 0);
        }
    }
    return new_postures;
}
}  // namespace kinematics
