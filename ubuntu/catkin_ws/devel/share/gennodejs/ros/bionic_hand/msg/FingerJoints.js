// Auto-generated. Do not edit!

// (in-package bionic_hand.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class FingerJoints {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.theta_M = null;
      this.theta_P = null;
      this.theta_D = null;
    }
    else {
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
    // Serializes a message object of type FingerJoints
    // Serialize message field [theta_M]
    bufferOffset = _serializer.float64(obj.theta_M, buffer, bufferOffset);
    // Serialize message field [theta_P]
    bufferOffset = _serializer.float64(obj.theta_P, buffer, bufferOffset);
    // Serialize message field [theta_D]
    bufferOffset = _serializer.float64(obj.theta_D, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FingerJoints
    let len;
    let data = new FingerJoints(null);
    // Deserialize message field [theta_M]
    data.theta_M = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [theta_P]
    data.theta_P = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [theta_D]
    data.theta_D = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'bionic_hand/FingerJoints';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '87bf9b04d1a94d2eda566ee32685c210';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new FingerJoints(null);
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

module.exports = FingerJoints;
