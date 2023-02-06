// Auto-generated. Do not edit!

// (in-package lidar_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let PointStamped = require('./PointStamped.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Lane {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.points = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('points')) {
        this.points = initObj.points
      }
      else {
        this.points = new Array(200).fill(new PointStamped());
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Lane
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [points] has the right length
    if (obj.points.length !== 200) {
      throw new Error('Unable to serialize array field points - length must be 200')
    }
    // Serialize message field [points]
    obj.points.forEach((val) => {
      bufferOffset = PointStamped.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Lane
    let len;
    let data = new Lane(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [points]
    len = 200;
    data.points = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.points[i] = PointStamped.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 6400;
  }

  static datatype() {
    // Returns string type for a message object
    return 'lidar_msgs/Lane';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '75af1c1affc9fe6f9de6b476e4c14f22';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    PointStamped[200] points
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
    string frame_id
    
    ================================================================================
    MSG: lidar_msgs/PointStamped
    float64 x
    float64 y
    float64 z
    float64 intensity
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Lane(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.points !== undefined) {
      resolved.points = new Array(200)
      for (let i = 0; i < resolved.points.length; ++i) {
        if (msg.points.length > i) {
          resolved.points[i] = PointStamped.Resolve(msg.points[i]);
        }
        else {
          resolved.points[i] = new PointStamped();
        }
      }
    }
    else {
      resolved.points = new Array(200).fill(new PointStamped())
    }

    return resolved;
    }
};

module.exports = Lane;
