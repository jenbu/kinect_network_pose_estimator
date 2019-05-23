// Auto-generated. Do not edit!

// (in-package pose_estimator.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class pipe_pose {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.pipe_x = null;
      this.pipe_y = null;
      this.pipe_z = null;
      this.pipe_theta_x = null;
      this.pipe_theta_y = null;
      this.pipe_theta_z = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('pipe_x')) {
        this.pipe_x = initObj.pipe_x
      }
      else {
        this.pipe_x = 0.0;
      }
      if (initObj.hasOwnProperty('pipe_y')) {
        this.pipe_y = initObj.pipe_y
      }
      else {
        this.pipe_y = 0.0;
      }
      if (initObj.hasOwnProperty('pipe_z')) {
        this.pipe_z = initObj.pipe_z
      }
      else {
        this.pipe_z = 0.0;
      }
      if (initObj.hasOwnProperty('pipe_theta_x')) {
        this.pipe_theta_x = initObj.pipe_theta_x
      }
      else {
        this.pipe_theta_x = 0.0;
      }
      if (initObj.hasOwnProperty('pipe_theta_y')) {
        this.pipe_theta_y = initObj.pipe_theta_y
      }
      else {
        this.pipe_theta_y = 0.0;
      }
      if (initObj.hasOwnProperty('pipe_theta_z')) {
        this.pipe_theta_z = initObj.pipe_theta_z
      }
      else {
        this.pipe_theta_z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type pipe_pose
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [pipe_x]
    bufferOffset = _serializer.float32(obj.pipe_x, buffer, bufferOffset);
    // Serialize message field [pipe_y]
    bufferOffset = _serializer.float32(obj.pipe_y, buffer, bufferOffset);
    // Serialize message field [pipe_z]
    bufferOffset = _serializer.float32(obj.pipe_z, buffer, bufferOffset);
    // Serialize message field [pipe_theta_x]
    bufferOffset = _serializer.float32(obj.pipe_theta_x, buffer, bufferOffset);
    // Serialize message field [pipe_theta_y]
    bufferOffset = _serializer.float32(obj.pipe_theta_y, buffer, bufferOffset);
    // Serialize message field [pipe_theta_z]
    bufferOffset = _serializer.float32(obj.pipe_theta_z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type pipe_pose
    let len;
    let data = new pipe_pose(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [pipe_x]
    data.pipe_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pipe_y]
    data.pipe_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pipe_z]
    data.pipe_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pipe_theta_x]
    data.pipe_theta_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pipe_theta_y]
    data.pipe_theta_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pipe_theta_z]
    data.pipe_theta_z = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pose_estimator/pipe_pose';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '51eaea2844bf3b987e1fb67e8d6ca8e2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float32 pipe_x
    float32 pipe_y
    float32 pipe_z
    float32 pipe_theta_x
    float32 pipe_theta_y
    float32 pipe_theta_z
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new pipe_pose(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.pipe_x !== undefined) {
      resolved.pipe_x = msg.pipe_x;
    }
    else {
      resolved.pipe_x = 0.0
    }

    if (msg.pipe_y !== undefined) {
      resolved.pipe_y = msg.pipe_y;
    }
    else {
      resolved.pipe_y = 0.0
    }

    if (msg.pipe_z !== undefined) {
      resolved.pipe_z = msg.pipe_z;
    }
    else {
      resolved.pipe_z = 0.0
    }

    if (msg.pipe_theta_x !== undefined) {
      resolved.pipe_theta_x = msg.pipe_theta_x;
    }
    else {
      resolved.pipe_theta_x = 0.0
    }

    if (msg.pipe_theta_y !== undefined) {
      resolved.pipe_theta_y = msg.pipe_theta_y;
    }
    else {
      resolved.pipe_theta_y = 0.0
    }

    if (msg.pipe_theta_z !== undefined) {
      resolved.pipe_theta_z = msg.pipe_theta_z;
    }
    else {
      resolved.pipe_theta_z = 0.0
    }

    return resolved;
    }
};

module.exports = pipe_pose;
