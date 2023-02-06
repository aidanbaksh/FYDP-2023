// Auto-generated. Do not edit!

// (in-package lidar_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let BoxInfo = require('./BoxInfo.js');

//-----------------------------------------------------------

class SocketBox {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.num_of_box = null;
      this.box_info = null;
    }
    else {
      if (initObj.hasOwnProperty('num_of_box')) {
        this.num_of_box = initObj.num_of_box
      }
      else {
        this.num_of_box = 0;
      }
      if (initObj.hasOwnProperty('box_info')) {
        this.box_info = initObj.box_info
      }
      else {
        this.box_info = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SocketBox
    // Serialize message field [num_of_box]
    bufferOffset = _serializer.int32(obj.num_of_box, buffer, bufferOffset);
    // Serialize message field [box_info]
    // Serialize the length for message field [box_info]
    bufferOffset = _serializer.uint32(obj.box_info.length, buffer, bufferOffset);
    obj.box_info.forEach((val) => {
      bufferOffset = BoxInfo.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SocketBox
    let len;
    let data = new SocketBox(null);
    // Deserialize message field [num_of_box]
    data.num_of_box = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [box_info]
    // Deserialize array length for message field [box_info]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.box_info = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.box_info[i] = BoxInfo.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.box_info.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'lidar_msgs/SocketBox';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7e7c1fed7bbc79bff735bfacc0aaebc3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 num_of_box
    BoxInfo[] box_info
    
    ================================================================================
    MSG: lidar_msgs/BoxInfo
    box box
    
    ================================================================================
    MSG: lidar_msgs/box
    float32 x_min
    float32 y_min
    float32 z_min
    float32 x_max
    float32 y_max
    float32 z_max
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SocketBox(null);
    if (msg.num_of_box !== undefined) {
      resolved.num_of_box = msg.num_of_box;
    }
    else {
      resolved.num_of_box = 0
    }

    if (msg.box_info !== undefined) {
      resolved.box_info = new Array(msg.box_info.length);
      for (let i = 0; i < resolved.box_info.length; ++i) {
        resolved.box_info[i] = BoxInfo.Resolve(msg.box_info[i]);
      }
    }
    else {
      resolved.box_info = []
    }

    return resolved;
    }
};

module.exports = SocketBox;
