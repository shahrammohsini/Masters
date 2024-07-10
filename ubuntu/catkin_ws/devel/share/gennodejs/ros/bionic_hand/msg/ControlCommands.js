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

class ControlCommands {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.PWM = null;
    }
    else {
      if (initObj.hasOwnProperty('PWM')) {
        this.PWM = initObj.PWM
      }
      else {
        this.PWM = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControlCommands
    // Serialize message field [PWM]
    bufferOffset = _serializer.float64(obj.PWM, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControlCommands
    let len;
    let data = new ControlCommands(null);
    // Deserialize message field [PWM]
    data.PWM = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'bionic_hand/ControlCommands';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '630d1348e66951f61746659ef3574616';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 PWM
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControlCommands(null);
    if (msg.PWM !== undefined) {
      resolved.PWM = msg.PWM;
    }
    else {
      resolved.PWM = 0.0
    }

    return resolved;
    }
};

module.exports = ControlCommands;
