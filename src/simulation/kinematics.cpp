#include "simulation/kinematics.h"

#include "Eigen/Dense"

#include "acclaim/bone.h"
#include "util/helper.h"

namespace kinematics {
void forwardSolver(const acclaim::Posture& posture, acclaim::Bone* bone) {
    // TODO
    // This function will be called with bone == root bone of the skeleton
    // You should set these variables:
    //     bone->start_position = Eigen::Vector4d::Zero();
    //     bone->end_position = Eigen::Vector4d::Zero();
    //     bone->rotation = Eigen::Matrix4d::Zero();
    // The sample above just set everything to zero
    
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

std::vector<acclaim::Posture> timeWarper(const std::vector<acclaim::Posture>& postures, int keyframe_old,
                                         int keyframe_new) {
    int total_frames = static_cast<int>(postures.size());
    int total_bones = static_cast<int>(postures[0].bone_rotations.size());
    std::vector<acclaim::Posture> new_postures = postures;
    for (int i = 0; i < total_frames; ++i) {
        for (int j = 0; j < total_bones; ++j) {
            // TODO
            // You should set these variables:
            //     new_postures[i].bone_translations[j] = postures[i].bone_translations[j];
            //     new_postures[i].bone_rotations[j] = postures[i].bone_rotations[j];
            // The sample above just change nothing
        }
    }
    return new_postures;
}
}  // namespace kinematics
