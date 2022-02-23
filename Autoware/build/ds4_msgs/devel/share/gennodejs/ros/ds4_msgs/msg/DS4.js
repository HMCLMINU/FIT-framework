// Auto-generated. Do not edit!

// (in-package ds4_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class DS4 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.square = null;
      this.cross = null;
      this.circle = null;
      this.triangle = null;
      this.up = null;
      this.right = null;
      this.down = null;
      this.left = null;
      this.l1 = null;
      this.r1 = null;
      this.l2 = null;
      this.r2 = null;
      this.l3 = null;
      this.r3 = null;
      this.left_x = null;
      this.left_y = null;
      this.right_x = null;
      this.right_y = null;
      this.share = null;
      this.option = null;
      this.ps = null;
      this.touchpad = null;
      this.acc_x = null;
      this.acc_y = null;
      this.acc_z = null;
      this.gyro_rol = null;
      this.gyro_pit = null;
      this.gyro_yaw = null;
      this.connected = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('square')) {
        this.square = initObj.square
      }
      else {
        this.square = false;
      }
      if (initObj.hasOwnProperty('cross')) {
        this.cross = initObj.cross
      }
      else {
        this.cross = false;
      }
      if (initObj.hasOwnProperty('circle')) {
        this.circle = initObj.circle
      }
      else {
        this.circle = false;
      }
      if (initObj.hasOwnProperty('triangle')) {
        this.triangle = initObj.triangle
      }
      else {
        this.triangle = false;
      }
      if (initObj.hasOwnProperty('up')) {
        this.up = initObj.up
      }
      else {
        this.up = false;
      }
      if (initObj.hasOwnProperty('right')) {
        this.right = initObj.right
      }
      else {
        this.right = false;
      }
      if (initObj.hasOwnProperty('down')) {
        this.down = initObj.down
      }
      else {
        this.down = false;
      }
      if (initObj.hasOwnProperty('left')) {
        this.left = initObj.left
      }
      else {
        this.left = false;
      }
      if (initObj.hasOwnProperty('l1')) {
        this.l1 = initObj.l1
      }
      else {
        this.l1 = false;
      }
      if (initObj.hasOwnProperty('r1')) {
        this.r1 = initObj.r1
      }
      else {
        this.r1 = false;
      }
      if (initObj.hasOwnProperty('l2')) {
        this.l2 = initObj.l2
      }
      else {
        this.l2 = 0.0;
      }
      if (initObj.hasOwnProperty('r2')) {
        this.r2 = initObj.r2
      }
      else {
        this.r2 = 0.0;
      }
      if (initObj.hasOwnProperty('l3')) {
        this.l3 = initObj.l3
      }
      else {
        this.l3 = false;
      }
      if (initObj.hasOwnProperty('r3')) {
        this.r3 = initObj.r3
      }
      else {
        this.r3 = false;
      }
      if (initObj.hasOwnProperty('left_x')) {
        this.left_x = initObj.left_x
      }
      else {
        this.left_x = 0.0;
      }
      if (initObj.hasOwnProperty('left_y')) {
        this.left_y = initObj.left_y
      }
      else {
        this.left_y = 0.0;
      }
      if (initObj.hasOwnProperty('right_x')) {
        this.right_x = initObj.right_x
      }
      else {
        this.right_x = 0.0;
      }
      if (initObj.hasOwnProperty('right_y')) {
        this.right_y = initObj.right_y
      }
      else {
        this.right_y = 0.0;
      }
      if (initObj.hasOwnProperty('share')) {
        this.share = initObj.share
      }
      else {
        this.share = false;
      }
      if (initObj.hasOwnProperty('option')) {
        this.option = initObj.option
      }
      else {
        this.option = false;
      }
      if (initObj.hasOwnProperty('ps')) {
        this.ps = initObj.ps
      }
      else {
        this.ps = false;
      }
      if (initObj.hasOwnProperty('touchpad')) {
        this.touchpad = initObj.touchpad
      }
      else {
        this.touchpad = false;
      }
      if (initObj.hasOwnProperty('acc_x')) {
        this.acc_x = initObj.acc_x
      }
      else {
        this.acc_x = 0.0;
      }
      if (initObj.hasOwnProperty('acc_y')) {
        this.acc_y = initObj.acc_y
      }
      else {
        this.acc_y = 0.0;
      }
      if (initObj.hasOwnProperty('acc_z')) {
        this.acc_z = initObj.acc_z
      }
      else {
        this.acc_z = 0.0;
      }
      if (initObj.hasOwnProperty('gyro_rol')) {
        this.gyro_rol = initObj.gyro_rol
      }
      else {
        this.gyro_rol = 0.0;
      }
      if (initObj.hasOwnProperty('gyro_pit')) {
        this.gyro_pit = initObj.gyro_pit
      }
      else {
        this.gyro_pit = 0.0;
      }
      if (initObj.hasOwnProperty('gyro_yaw')) {
        this.gyro_yaw = initObj.gyro_yaw
      }
      else {
        this.gyro_yaw = 0.0;
      }
      if (initObj.hasOwnProperty('connected')) {
        this.connected = initObj.connected
      }
      else {
        this.connected = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DS4
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [square]
    bufferOffset = _serializer.bool(obj.square, buffer, bufferOffset);
    // Serialize message field [cross]
    bufferOffset = _serializer.bool(obj.cross, buffer, bufferOffset);
    // Serialize message field [circle]
    bufferOffset = _serializer.bool(obj.circle, buffer, bufferOffset);
    // Serialize message field [triangle]
    bufferOffset = _serializer.bool(obj.triangle, buffer, bufferOffset);
    // Serialize message field [up]
    bufferOffset = _serializer.bool(obj.up, buffer, bufferOffset);
    // Serialize message field [right]
    bufferOffset = _serializer.bool(obj.right, buffer, bufferOffset);
    // Serialize message field [down]
    bufferOffset = _serializer.bool(obj.down, buffer, bufferOffset);
    // Serialize message field [left]
    bufferOffset = _serializer.bool(obj.left, buffer, bufferOffset);
    // Serialize message field [l1]
    bufferOffset = _serializer.bool(obj.l1, buffer, bufferOffset);
    // Serialize message field [r1]
    bufferOffset = _serializer.bool(obj.r1, buffer, bufferOffset);
    // Serialize message field [l2]
    bufferOffset = _serializer.float64(obj.l2, buffer, bufferOffset);
    // Serialize message field [r2]
    bufferOffset = _serializer.float64(obj.r2, buffer, bufferOffset);
    // Serialize message field [l3]
    bufferOffset = _serializer.bool(obj.l3, buffer, bufferOffset);
    // Serialize message field [r3]
    bufferOffset = _serializer.bool(obj.r3, buffer, bufferOffset);
    // Serialize message field [left_x]
    bufferOffset = _serializer.float64(obj.left_x, buffer, bufferOffset);
    // Serialize message field [left_y]
    bufferOffset = _serializer.float64(obj.left_y, buffer, bufferOffset);
    // Serialize message field [right_x]
    bufferOffset = _serializer.float64(obj.right_x, buffer, bufferOffset);
    // Serialize message field [right_y]
    bufferOffset = _serializer.float64(obj.right_y, buffer, bufferOffset);
    // Serialize message field [share]
    bufferOffset = _serializer.bool(obj.share, buffer, bufferOffset);
    // Serialize message field [option]
    bufferOffset = _serializer.bool(obj.option, buffer, bufferOffset);
    // Serialize message field [ps]
    bufferOffset = _serializer.bool(obj.ps, buffer, bufferOffset);
    // Serialize message field [touchpad]
    bufferOffset = _serializer.bool(obj.touchpad, buffer, bufferOffset);
    // Serialize message field [acc_x]
    bufferOffset = _serializer.float64(obj.acc_x, buffer, bufferOffset);
    // Serialize message field [acc_y]
    bufferOffset = _serializer.float64(obj.acc_y, buffer, bufferOffset);
    // Serialize message field [acc_z]
    bufferOffset = _serializer.float64(obj.acc_z, buffer, bufferOffset);
    // Serialize message field [gyro_rol]
    bufferOffset = _serializer.float64(obj.gyro_rol, buffer, bufferOffset);
    // Serialize message field [gyro_pit]
    bufferOffset = _serializer.float64(obj.gyro_pit, buffer, bufferOffset);
    // Serialize message field [gyro_yaw]
    bufferOffset = _serializer.float64(obj.gyro_yaw, buffer, bufferOffset);
    // Serialize message field [connected]
    bufferOffset = _serializer.bool(obj.connected, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DS4
    let len;
    let data = new DS4(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [square]
    data.square = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [cross]
    data.cross = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [circle]
    data.circle = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [triangle]
    data.triangle = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [up]
    data.up = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [right]
    data.right = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [down]
    data.down = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [left]
    data.left = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [l1]
    data.l1 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [r1]
    data.r1 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [l2]
    data.l2 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [r2]
    data.r2 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [l3]
    data.l3 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [r3]
    data.r3 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [left_x]
    data.left_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [left_y]
    data.left_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [right_x]
    data.right_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [right_y]
    data.right_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [share]
    data.share = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [option]
    data.option = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ps]
    data.ps = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [touchpad]
    data.touchpad = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [acc_x]
    data.acc_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [acc_y]
    data.acc_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [acc_z]
    data.acc_z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [gyro_rol]
    data.gyro_rol = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [gyro_pit]
    data.gyro_pit = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [gyro_yaw]
    data.gyro_yaw = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [connected]
    data.connected = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 113;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ds4_msgs/DS4';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9c17d8a8df3d9463ab329c13d2358031';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    bool square
    bool cross
    bool circle
    bool triangle
    
    bool up
    bool right
    bool down
    bool left
    
    bool l1
    bool r1
    float64 l2
    float64 r2
    bool l3
    bool r3
    
    float64 left_x
    float64 left_y
    float64 right_x
    float64 right_y
    
    bool share
    bool option
    bool ps
    bool touchpad
    
    float64 acc_x
    float64 acc_y
    float64 acc_z
    
    float64 gyro_rol
    float64 gyro_pit
    float64 gyro_yaw
    
    bool connected
    
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
    const resolved = new DS4(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.square !== undefined) {
      resolved.square = msg.square;
    }
    else {
      resolved.square = false
    }

    if (msg.cross !== undefined) {
      resolved.cross = msg.cross;
    }
    else {
      resolved.cross = false
    }

    if (msg.circle !== undefined) {
      resolved.circle = msg.circle;
    }
    else {
      resolved.circle = false
    }

    if (msg.triangle !== undefined) {
      resolved.triangle = msg.triangle;
    }
    else {
      resolved.triangle = false
    }

    if (msg.up !== undefined) {
      resolved.up = msg.up;
    }
    else {
      resolved.up = false
    }

    if (msg.right !== undefined) {
      resolved.right = msg.right;
    }
    else {
      resolved.right = false
    }

    if (msg.down !== undefined) {
      resolved.down = msg.down;
    }
    else {
      resolved.down = false
    }

    if (msg.left !== undefined) {
      resolved.left = msg.left;
    }
    else {
      resolved.left = false
    }

    if (msg.l1 !== undefined) {
      resolved.l1 = msg.l1;
    }
    else {
      resolved.l1 = false
    }

    if (msg.r1 !== undefined) {
      resolved.r1 = msg.r1;
    }
    else {
      resolved.r1 = false
    }

    if (msg.l2 !== undefined) {
      resolved.l2 = msg.l2;
    }
    else {
      resolved.l2 = 0.0
    }

    if (msg.r2 !== undefined) {
      resolved.r2 = msg.r2;
    }
    else {
      resolved.r2 = 0.0
    }

    if (msg.l3 !== undefined) {
      resolved.l3 = msg.l3;
    }
    else {
      resolved.l3 = false
    }

    if (msg.r3 !== undefined) {
      resolved.r3 = msg.r3;
    }
    else {
      resolved.r3 = false
    }

    if (msg.left_x !== undefined) {
      resolved.left_x = msg.left_x;
    }
    else {
      resolved.left_x = 0.0
    }

    if (msg.left_y !== undefined) {
      resolved.left_y = msg.left_y;
    }
    else {
      resolved.left_y = 0.0
    }

    if (msg.right_x !== undefined) {
      resolved.right_x = msg.right_x;
    }
    else {
      resolved.right_x = 0.0
    }

    if (msg.right_y !== undefined) {
      resolved.right_y = msg.right_y;
    }
    else {
      resolved.right_y = 0.0
    }

    if (msg.share !== undefined) {
      resolved.share = msg.share;
    }
    else {
      resolved.share = false
    }

    if (msg.option !== undefined) {
      resolved.option = msg.option;
    }
    else {
      resolved.option = false
    }

    if (msg.ps !== undefined) {
      resolved.ps = msg.ps;
    }
    else {
      resolved.ps = false
    }

    if (msg.touchpad !== undefined) {
      resolved.touchpad = msg.touchpad;
    }
    else {
      resolved.touchpad = false
    }

    if (msg.acc_x !== undefined) {
      resolved.acc_x = msg.acc_x;
    }
    else {
      resolved.acc_x = 0.0
    }

    if (msg.acc_y !== undefined) {
      resolved.acc_y = msg.acc_y;
    }
    else {
      resolved.acc_y = 0.0
    }

    if (msg.acc_z !== undefined) {
      resolved.acc_z = msg.acc_z;
    }
    else {
      resolved.acc_z = 0.0
    }

    if (msg.gyro_rol !== undefined) {
      resolved.gyro_rol = msg.gyro_rol;
    }
    else {
      resolved.gyro_rol = 0.0
    }

    if (msg.gyro_pit !== undefined) {
      resolved.gyro_pit = msg.gyro_pit;
    }
    else {
      resolved.gyro_pit = 0.0
    }

    if (msg.gyro_yaw !== undefined) {
      resolved.gyro_yaw = msg.gyro_yaw;
    }
    else {
      resolved.gyro_yaw = 0.0
    }

    if (msg.connected !== undefined) {
      resolved.connected = msg.connected;
    }
    else {
      resolved.connected = false
    }

    return resolved;
    }
};

module.exports = DS4;
