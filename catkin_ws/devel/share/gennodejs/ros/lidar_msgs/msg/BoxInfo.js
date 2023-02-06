// Auto-generated. Do not edit!

// (in-package lidar_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let box = require('./box.js');

//-----------------------------------------------------------

class BoxInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.box = null;
    }
    else {
      if (initObj.hasOwnProperty('box')) {
        this.box = initObj.box
      }
      else {
        this.box = new box();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BoxInfo
    // Serialize message field [box]
    bufferOffset = box.serialize(obj.box, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BoxInfo
    let len;
    let data = new BoxInfo(null);
    // Deserialize message field [box]
    data.box = box.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'lidar_msgs/BoxInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7f621b8f09e614f07fb08a2b7f9a6583';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new BoxInfo(null);
    if (msg.box !== undefined) {
      resolved.box = box.Resolve(msg.box)
    }
    else {
      resolved.box = new box()
    }

    return resolved;
    }
};

module.exports = BoxInfo;
