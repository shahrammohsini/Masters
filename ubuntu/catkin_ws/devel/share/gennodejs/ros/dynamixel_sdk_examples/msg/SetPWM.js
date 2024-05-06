// Auto-generated. Do not edit!

// (in-package dynamixel_sdk_examples.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class SetPWM {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.pwm = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('pwm')) {
        this.pwm = initObj.pwm
      }
      else {
        this.pwm = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetPWM
    // Serialize message field [id]
    bufferOffset = _serializer.uint8(obj.id, buffer, bufferOffset);
    // Serialize message field [pwm]
    bufferOffset = _serializer.int32(obj.pwm, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetPWM
    let len;
    let data = new SetPWM(null);
    // Deserialize message field [id]
    data.id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [pwm]
    data.pwm = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dynamixel_sdk_examples/SetPWM';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7dcb9d04de9857b62a1d688544986dcd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 id
    int32 pwm
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetPWM(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.pwm !== undefined) {
      resolved.pwm = msg.pwm;
    }
    else {
      resolved.pwm = 0
    }

    return resolved;
    }
};

module.exports = SetPWM;
