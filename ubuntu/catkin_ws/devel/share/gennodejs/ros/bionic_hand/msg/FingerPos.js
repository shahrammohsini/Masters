// Auto-generated. Do not edit!

// (in-package bionic_hand.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class FingerPos {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.theta_M = null;
      this.theta_P = null;
      this.theta_D = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('theta_M')) {
        this.theta_M = initObj.theta_M
      }
      else {
        this.theta_M = 0.0;
      }
      if (initObj.hasOwnProperty('theta_P')) {
        this.theta_P = initObj.theta_P
      }
      else {
        this.theta_P = 0.0;
      }
      if (initObj.hasOwnProperty('theta_D')) {
        this.theta_D = initObj.theta_D
      }
      else {
        this.theta_D = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FingerPos
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [theta_M]
    bufferOffset = _serializer.float64(obj.theta_M, buffer, bufferOffset);
    // Serialize message field [theta_P]
    bufferOffset = _serializer.float64(obj.theta_P, buffer, bufferOffset);
    // Serialize message field [theta_D]
    bufferOffset = _serializer.float64(obj.theta_D, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FingerPos
    let len;
    let data = new FingerPos(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [theta_M]
    data.theta_M = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [theta_P]
    data.theta_P = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [theta_D]
    data.theta_D = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'bionic_hand/FingerPos';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4b76b67765bb2cfec63fff0018dee699';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64 theta_M
    float64 theta_P
    float64 theta_D
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

    if (msg.theta_M !== undefined) {
      resolved.theta_M = msg.theta_M;
    }
    else {
      resolved.theta_M = 0.0
    }

    if (msg.theta_P !== undefined) {
      resolved.theta_P = msg.theta_P;
    }
    else {
      resolved.theta_P = 0.0
    }

    if (msg.theta_D !== undefined) {
      resolved.theta_D = msg.theta_D;
    }
    else {
      resolved.theta_D = 0.0
    }

    return resolved;
    }
};

module.exports = FingerPos;
