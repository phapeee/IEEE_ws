// Auto-generated. Do not edit!

// (in-package april_tag_detection.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class GetAprilTagRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.tag_id = null;
      this.cam_id = null;
    }
    else {
      if (initObj.hasOwnProperty('tag_id')) {
        this.tag_id = initObj.tag_id
      }
      else {
        this.tag_id = '';
      }
      if (initObj.hasOwnProperty('cam_id')) {
        this.cam_id = initObj.cam_id
      }
      else {
        this.cam_id = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetAprilTagRequest
    // Serialize message field [tag_id]
    bufferOffset = _serializer.string(obj.tag_id, buffer, bufferOffset);
    // Serialize message field [cam_id]
    bufferOffset = _serializer.string(obj.cam_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetAprilTagRequest
    let len;
    let data = new GetAprilTagRequest(null);
    // Deserialize message field [tag_id]
    data.tag_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [cam_id]
    data.cam_id = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.tag_id);
    length += _getByteLength(object.cam_id);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'april_tag_detection/GetAprilTagRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9e850c675d61239dfbb5e7f7d9441439';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string tag_id
    string cam_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetAprilTagRequest(null);
    if (msg.tag_id !== undefined) {
      resolved.tag_id = msg.tag_id;
    }
    else {
      resolved.tag_id = ''
    }

    if (msg.cam_id !== undefined) {
      resolved.cam_id = msg.cam_id;
    }
    else {
      resolved.cam_id = ''
    }

    return resolved;
    }
};

class GetAprilTagResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.tag_count = null;
      this.tag_id = null;
      this.cam_id = null;
      this.poses = null;
    }
    else {
      if (initObj.hasOwnProperty('tag_count')) {
        this.tag_count = initObj.tag_count
      }
      else {
        this.tag_count = 0;
      }
      if (initObj.hasOwnProperty('tag_id')) {
        this.tag_id = initObj.tag_id
      }
      else {
        this.tag_id = [];
      }
      if (initObj.hasOwnProperty('cam_id')) {
        this.cam_id = initObj.cam_id
      }
      else {
        this.cam_id = [];
      }
      if (initObj.hasOwnProperty('poses')) {
        this.poses = initObj.poses
      }
      else {
        this.poses = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetAprilTagResponse
    // Serialize message field [tag_count]
    bufferOffset = _serializer.uint8(obj.tag_count, buffer, bufferOffset);
    // Serialize message field [tag_id]
    bufferOffset = _arraySerializer.uint8(obj.tag_id, buffer, bufferOffset, null);
    // Serialize message field [cam_id]
    bufferOffset = _arraySerializer.uint8(obj.cam_id, buffer, bufferOffset, null);
    // Serialize message field [poses]
    // Serialize the length for message field [poses]
    bufferOffset = _serializer.uint32(obj.poses.length, buffer, bufferOffset);
    obj.poses.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Pose.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetAprilTagResponse
    let len;
    let data = new GetAprilTagResponse(null);
    // Deserialize message field [tag_count]
    data.tag_count = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [tag_id]
    data.tag_id = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [cam_id]
    data.cam_id = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [poses]
    // Deserialize array length for message field [poses]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.poses = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.poses[i] = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.tag_id.length;
    length += object.cam_id.length;
    length += 56 * object.poses.length;
    return length + 13;
  }

  static datatype() {
    // Returns string type for a service object
    return 'april_tag_detection/GetAprilTagResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a747909d241398506309a0b18c8aa9eb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 tag_count
    uint8[] tag_id
    uint8[] cam_id
    geometry_msgs/Pose[] poses
    
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetAprilTagResponse(null);
    if (msg.tag_count !== undefined) {
      resolved.tag_count = msg.tag_count;
    }
    else {
      resolved.tag_count = 0
    }

    if (msg.tag_id !== undefined) {
      resolved.tag_id = msg.tag_id;
    }
    else {
      resolved.tag_id = []
    }

    if (msg.cam_id !== undefined) {
      resolved.cam_id = msg.cam_id;
    }
    else {
      resolved.cam_id = []
    }

    if (msg.poses !== undefined) {
      resolved.poses = new Array(msg.poses.length);
      for (let i = 0; i < resolved.poses.length; ++i) {
        resolved.poses[i] = geometry_msgs.msg.Pose.Resolve(msg.poses[i]);
      }
    }
    else {
      resolved.poses = []
    }

    return resolved;
    }
};

module.exports = {
  Request: GetAprilTagRequest,
  Response: GetAprilTagResponse,
  md5sum() { return '11adae4fdc00ea7e90b1c993476c8886'; },
  datatype() { return 'april_tag_detection/GetAprilTag'; }
};
