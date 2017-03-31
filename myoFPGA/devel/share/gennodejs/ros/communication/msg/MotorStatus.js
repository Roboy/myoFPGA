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

class MotorStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pwmRef = null;
      this.position = null;
      this.velocity = null;
      this.displacement = null;
      this.current = null;
    }
    else {
      if (initObj.hasOwnProperty('pwmRef')) {
        this.pwmRef = initObj.pwmRef
      }
      else {
        this.pwmRef = [];
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = [];
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = [];
      }
      if (initObj.hasOwnProperty('displacement')) {
        this.displacement = initObj.displacement
      }
      else {
        this.displacement = [];
      }
      if (initObj.hasOwnProperty('current')) {
        this.current = initObj.current
      }
      else {
        this.current = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotorStatus
    // Serialize message field [pwmRef]
    bufferOffset = _arraySerializer.int32(obj.pwmRef, buffer, bufferOffset, null);
    // Serialize message field [position]
    bufferOffset = _arraySerializer.int32(obj.position, buffer, bufferOffset, null);
    // Serialize message field [velocity]
    bufferOffset = _arraySerializer.int16(obj.velocity, buffer, bufferOffset, null);
    // Serialize message field [displacement]
    bufferOffset = _arraySerializer.int16(obj.displacement, buffer, bufferOffset, null);
    // Serialize message field [current]
    bufferOffset = _arraySerializer.int16(obj.current, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotorStatus
    let len;
    let data = new MotorStatus(null);
    // Deserialize message field [pwmRef]
    data.pwmRef = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [position]
    data.position = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [velocity]
    data.velocity = _arrayDeserializer.int16(buffer, bufferOffset, null)
    // Deserialize message field [displacement]
    data.displacement = _arrayDeserializer.int16(buffer, bufferOffset, null)
    // Deserialize message field [current]
    data.current = _arrayDeserializer.int16(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.pwmRef.length;
    length += 4 * object.position.length;
    length += 2 * object.velocity.length;
    length += 2 * object.displacement.length;
    length += 2 * object.current.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'communication/MotorStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8e3cb2e63688fce1513f561de7a5ce50';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32[] pwmRef
    int32[] position
    int16[] velocity
    int16[] displacement
    int16[] current
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MotorStatus(null);
    if (msg.pwmRef !== undefined) {
      resolved.pwmRef = msg.pwmRef;
    }
    else {
      resolved.pwmRef = []
    }

    if (msg.position !== undefined) {
      resolved.position = msg.position;
    }
    else {
      resolved.position = []
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = msg.velocity;
    }
    else {
      resolved.velocity = []
    }

    if (msg.displacement !== undefined) {
      resolved.displacement = msg.displacement;
    }
    else {
      resolved.displacement = []
    }

    if (msg.current !== undefined) {
      resolved.current = msg.current;
    }
    else {
      resolved.current = []
    }

    return resolved;
    }
};

module.exports = MotorStatus;
