import io
import os
from typing import List

import rospy
from geometry_msgs.msg import TransformStamped

N_TRANSFORMS: int = 4
CALIBRATED_TRANSFORMS_DIR = os.path.join(os.path.expanduser("~"), '.ros', 'tf_calibrated')

# make CALIBRATED_TRANSFORMS_DIR if it does not exist
if not os.path.exists(CALIBRATED_TRANSFORMS_DIR):
   os.makedirs(CALIBRATED_TRANSFORMS_DIR)
   rospy.loginfo("Created transforms directory %s", CALIBRATED_TRANSFORMS_DIR)


def _format_file_name(tf: TransformStamped) -> str:
    return os.path.join(
        CALIBRATED_TRANSFORMS_DIR,
        '{parent}_to_{child}.tf'.format(parent=tf.header.frame_id, child=tf.child_frame_id)
    )


def write(transforms: List[TransformStamped]) -> None:
    assert(len(transforms) == N_TRANSFORMS)

    for tf in transforms:
        # serialize each transform
        buf = io.BytesIO()
        tf.serialize(buf)

        # write to file
        with open(_format_file_name(tf), 'wb') as f:
            f.write(buf.getbuffer())
        
    rospy.loginfo("Saved transforms to disk!")


def read() -> List[TransformStamped]:
    stored_transforms = []

    for filename in os.listdir(CALIBRATED_TRANSFORMS_DIR):
        transfrom_file = os.path.join(CALIBRATED_TRANSFORMS_DIR, filename)
        assert(os.path.isfile(transfrom_file))  # should only be transform file in directory

        with open(transfrom_file, 'rb') as f:
            tf = TransformStamped()
            tf_serialized_bytes = f.read()
            tf.deserialize(tf_serialized_bytes)
            stored_transforms.append(tf)

    assert(len(stored_transforms) == N_TRANSFORMS)
    rospy.loginfo("Read transforms from disk")
        
    return stored_transforms
