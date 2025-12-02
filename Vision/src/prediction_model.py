#!/usr/bin/env python3
from tensorflow.keras.layers import Dense, Input, Flatten, Dropout
from tensorflow.keras.models import Sequential, Model
from tensorflow.keras.utils import plot_model
import numpy as np
import tensorflow as tf
from tensorflow.keras import layers, models
from scipy.spatial.transform import Rotation as R
from vision_detection.msg import PlaneResults, prediction_data
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
import math
from vision_detection.msg import prediction_data

model_data_pub = rospy.Publisher("/model_data", prediction_data, queue_size = 10)

def prediction_callback(data):
    system_input = np.array([data.input_data])
    reference_output = np.array([data.output_data])

    # 모델 검증
    #validate_model_prediction(model, system_input, reference_output)

    pred_pos, pred_rot = model.predict({"position_input" : system_input[:, :3], "rotation_input" : system_input[:, 3:]})

    pred_rot = rotation_6d_to_quaternion(pred_rot)
    pred_rot = standardize_quaternion(pred_rot)
    pred_out = np.hstack([pred_pos, pred_rot])
    pred_out = recover_output_rotation_prediction(pred_out)

    reference_output = recover_output_rotation_prediction(reference_output)

    #print(quaternion_euler_axis_error(rotation_6d_to_quaternion(system_input[:, 7:]), reference_output[:, 3:]))
    #print(quaternion_euler_axis_error(pred_rot, reference_output[:, 3:]))

    print(pred_out)
    print(reference_output)

    model_data = prediction_data()
    model_data.input_data = pred_out[0].tolist()
    model_data.output_data = reference_output[0].tolist()

    model_data_pub.publish(model_data)

    pred_pos = np.array([pred_out[0][:3]])
    pred_rot = np.array([pred_out[0][3:]])

    broadcaster = tf2_ros.StaticTransformBroadcaster()

    prediction = geometry_msgs.msg.TransformStamped()

    prediction.header.stamp = rospy.Time.now()
    prediction.header.frame_id = "camera_color_optical_frame"
    prediction.child_frame_id = "predicted_Object"

    prediction.transform.translation.x = pred_pos[0][0] / 1000
    prediction.transform.translation.y = pred_pos[0][1] / 1000
    prediction.transform.translation.z = pred_pos[0][2] / 1000

    prediction.transform.rotation.x = pred_rot[0][0]
    prediction.transform.rotation.y = pred_rot[0][1]
    prediction.transform.rotation.z = pred_rot[0][2]
    prediction.transform.rotation.w = pred_rot[0][3]

    broadcaster.sendTransform(prediction)
    
def position_loss(y_true, y_pred):
    return tf.sqrt(tf.reduce_mean(tf.square(y_true - y_pred)))

def tf_geodesic_loss(y_true, y_pred):
    """
    6D 회전 예측에 대한 지오데식 손실
    """
    R1 = tf_rotation_6d_to_matrix(y_true)
    R2 = tf_rotation_6d_to_matrix(y_pred)

    R_diff = tf.matmul(R1, R2, transpose_a=True)
    trace = tf.linalg.trace(R_diff)
    cos_theta = tf.clip_by_value((trace - 1.0) / 2.0, -1.0, 1.0)
    angle = tf.acos(cos_theta)
    return tf.reduce_mean(angle)

pred_data_sub = rospy.Subscriber("/prediction_data", prediction_data, prediction_callback)
model = tf.keras.models.load_model("/root/catkin_ws/src/vision_detection/vision_detection_weight/marker_predict_6.0", 
                                    custom_objects={
                                        "position_loss": position_loss, 
                                        "tf_geodesic_loss" : tf_geodesic_loss})

def quaternion_to_6d(q_array):
    """
    (N, 4) → (N, 6) 회전 표현
    """
    rot = R.from_quat(q_array)
    matrix = rot.as_matrix()  # (N, 3, 3)
    return matrix[:, :, :2].reshape(-1, 6)  # 앞의 2열만 사용

def tf_rotation_6d_to_matrix(x):
    """
    (N, 6) → (N, 3, 3) 회전행렬 복원
    """
    x = tf.reshape(x, [-1, 3, 2])
    b1 = tf.linalg.l2_normalize(x[:, :, 0], axis=1)
    b2 = tf.linalg.l2_normalize(
        x[:, :, 1] - tf.reduce_sum(b1 * x[:, :, 1], axis=1, keepdims=True) * b1,
        axis=1,
    )
    b3 = tf.linalg.cross(b1, b2)
    return tf.stack([b1, b2, b3], axis=-1)

    
def recover_output_rotation_prediction(outputs_pred):
    R_x_180 = R.from_euler('x', 180, degrees=True)
    outputs_restored = outputs_pred.copy()

    # 3. 출력 quaternion 복원
    q2 = outputs_pred[:, 3:7]
    r2 = R.from_quat(q2)
    q2_restored = (R_x_180 * r2).as_quat()

    # 4. 반영
    outputs_restored[:, 3:7] = q2_restored

    return outputs_restored

def standardize_quaternion(q):
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    if q.ndim == 1:
        return q if q[3] >= 0 else -q
    sign = np.where(q[:, 3:4] >= 0, 1.0, -1.0)
    return q * sign

def rotation_6d_to_quaternion(x_6d):
    """
    회전 6D → 쿼터니언 (for evaluation)
    """
    rot_matrix = tf_rotation_6d_to_matrix(tf.constant(x_6d, dtype=tf.float32))
    rot_matrix_np = rot_matrix.numpy()
    rot = R.from_matrix(rot_matrix_np)
    return rot.as_quat()


def quaternion_euler_axis_error(q1, q2):
    q1 = standardize_quaternion(q1)
    q2 = standardize_quaternion(q2)
    euler1 = R.from_quat(q1).as_euler('xyz', degrees=True)
    euler2 = R.from_quat(q2).as_euler('xyz', degrees=True)
    error = (euler1 - euler2 + 180) % 360 - 180
    return np.abs(error)  # (N, 3): [roll, pitch, yaw]

def validate_model_prediction(model, system_input, reference_output):
    """
    - system_input: 시스템에서 사용된 입력 (ex: 센서/ROS에서 추출된 실시간 입력)
    - reference_output: 해당 입력에 대한 실제 레이블 또는 모델 추론 기준값
    """

    # ✅ 모델 추론 비교
    pred_pos, pred_rot = model.predict({"position_input" : system_input[:, :3], "rotation_input" : system_input[:, 3:]})
    
    pred_rot = rotation_6d_to_quaternion(pred_rot)
    pred_rot = recover_output_rotation_prediction(pred_rot)
    ref_output = recover_output_rotation_prediction(reference_output)


    # 위치 비교
    pred_system = np.hstack([pred_pos, pred_rot])
    pos_sys = pred_system[:, :3]
    pos_ref = ref_output[:, :3]
    pos_diff = np.linalg.norm(pos_sys - pos_ref, axis=1)

    # 회전 비교
    q_sys = pred_system[:, 3:]
    q_ref = ref_output[:, 3:]
    rot_diff = quaternion_euler_axis_error(q_sys, q_ref)

    print("\n📍 위치 오차 (mm):", pos_diff)
    print("🔄 회전 오차 (deg):")
    print(" - Roll 평균:", np.mean(rot_diff[:, 0]))
    print(" - Pitch 평균:", np.mean(rot_diff[:, 1]))
    print(" - Yaw 평균:", np.mean(rot_diff[:, 2]))

if __name__ == "__main__":
    rospy.init_node('model_test', anonymous=True)
    rospy.spin()
