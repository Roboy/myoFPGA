// Auto-generated. Do not edit!

// (in-package communication.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class MotorCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.motors = null;
      this.setPoints = null;
    }
    else {
      if (initObj.hasOwnProperty('motors')) {
        this.motors = initObj.motors
      }
      else {
        this.motors = [];
      }
      if (initObj.hasOwnProperty('setPoints')) {
        this.setPoints = initObj.setPoints
      }
      else {
        this.setPoints = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotorCommand
    // Serialize message field [motors]
    bufferOffset = _arraySerializer.uint8(obj.motors, buffer, bufferOffset, null);
    // Serialize message field [setPoints]
    bufferOffset = _arraySerializer.int32(obj.setPoints, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotorCommand
    let len;
    let data = new MotorCommand(null);
    // Deserialize message field [motors]
    data.motors = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [setPoints]
    data.setPoints = _arrayDeserializer.int32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.motors.length;
    length += 4 * object.setPoints.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'communication/MotorCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7da1664b5fc700acbf0efa5165c4eb21';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8[] motors
    int32[] setPoints
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MotorCommand(null);
    if (msg.motors !== undefined) {
      resolved.motors = msg.motors;
    }
    else {
      resolved.motors = []
    }

    if (msg.setPoints !== undefined) {
      resolved.setPoints = msg.setPoints;
    }
    else {
      resolved.setPoints = []
    }

    return resolved;
    }
};

module.exports = MotorCommand;
