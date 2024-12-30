// Auto-generated. Do not edit!

// (in-package bionic_hand.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let FingerJoints = require('./FingerJoints.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class FingerPos {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.index = null;
      this.middle = null;
      this.thumb = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('index')) {
        this.index = initObj.index
      }
      else {
        this.index = new FingerJoints();
      }
      if (initObj.hasOwnProperty('middle')) {
        this.middle = initObj.middle
      }
      else {
        this.middle = new FingerJoints();
      }
      if (initObj.hasOwnProperty('thumb')) {
        this.thumb = initObj.thumb
      }
      else {
        this.thumb = new FingerJoints();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FingerPos
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [index]
    bufferOffset = FingerJoints.serialize(obj.index, buffer, bufferOffset);
    // Serialize message field [middle]
    bufferOffset = FingerJoints.serialize(obj.middle, buffer, bufferOffset);
    // Serialize message field [thumb]
    bufferOffset = FingerJoints.serialize(obj.thumb, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FingerPos
    let len;
    let data = new FingerPos(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [index]
    data.index = FingerJoints.deserialize(buffer, bufferOffset);
    // Deserialize message field [middle]
    data.middle = FingerJoints.deserialize(buffer, bufferOffset);
    // Deserialize message field [thumb]
    data.thumb = FingerJoints.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 72;
  }

  static datatype() {
    // Returns string type for a message object
    return 'bionic_hand/FingerPos';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ddb23831c46fbe010de19d051a0b3b5e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    FingerJoints index
    FingerJoints middle
    FingerJoints thumb
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
    MSG: bionic_hand/FingerJoints
    float64 theta_M
    float64 theta_P
    float64 theta_D
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FingerPos(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.index !== undefined) {
      resolved.index = FingerJoints.Resolve(msg.index)
    }
    else {
      resolved.index = new FingerJoints()
    }

    if (msg.middle !== undefined) {
      resolved.middle = FingerJoints.Resolve(msg.middle)
    }
    else {
      resolved.middle = new FingerJoints()
    }

    if (msg.thumb !== undefined) {
      resolved.thumb = FingerJoints.Resolve(msg.thumb)
    }
    else {
      resolved.thumb = new FingerJoints()
    }

    return resolved;
    }
};

module.exports = FingerPos;
