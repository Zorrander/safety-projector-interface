// Auto-generated. Do not edit!

// (in-package tuni_whitegoods_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class PointsDepthMapRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.poi_rgb = null;
    }
    else {
      if (initObj.hasOwnProperty('poi_rgb')) {
        this.poi_rgb = initObj.poi_rgb
      }
      else {
        this.poi_rgb = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PointsDepthMapRequest
    // Serialize message field [poi_rgb]
    // Serialize the length for message field [poi_rgb]
    bufferOffset = _serializer.uint32(obj.poi_rgb.length, buffer, bufferOffset);
    obj.poi_rgb.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PointsDepthMapRequest
    let len;
    let data = new PointsDepthMapRequest(null);
    // Deserialize message field [poi_rgb]
    // Deserialize array length for message field [poi_rgb]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.poi_rgb = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.poi_rgb[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.poi_rgb.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tuni_whitegoods_msgs/PointsDepthMapRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6ecbdb8dda7de62ad2fecb489d597c09';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Point[] poi_rgb
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PointsDepthMapRequest(null);
    if (msg.poi_rgb !== undefined) {
      resolved.poi_rgb = new Array(msg.poi_rgb.length);
      for (let i = 0; i < resolved.poi_rgb.length; ++i) {
        resolved.poi_rgb[i] = geometry_msgs.msg.Point.Resolve(msg.poi_rgb[i]);
      }
    }
    else {
      resolved.poi_rgb = []
    }

    return resolved;
    }
};

class PointsDepthMapResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.poi_depth_map = null;
    }
    else {
      if (initObj.hasOwnProperty('poi_depth_map')) {
        this.poi_depth_map = initObj.poi_depth_map
      }
      else {
        this.poi_depth_map = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PointsDepthMapResponse
    // Serialize message field [poi_depth_map]
    // Serialize the length for message field [poi_depth_map]
    bufferOffset = _serializer.uint32(obj.poi_depth_map.length, buffer, bufferOffset);
    obj.poi_depth_map.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PointsDepthMapResponse
    let len;
    let data = new PointsDepthMapResponse(null);
    // Deserialize message field [poi_depth_map]
    // Deserialize array length for message field [poi_depth_map]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.poi_depth_map = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.poi_depth_map[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.poi_depth_map.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tuni_whitegoods_msgs/PointsDepthMapResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a14e9f9688e30d57ed425a9c43605081';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Point[] poi_depth_map
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PointsDepthMapResponse(null);
    if (msg.poi_depth_map !== undefined) {
      resolved.poi_depth_map = new Array(msg.poi_depth_map.length);
      for (let i = 0; i < resolved.poi_depth_map.length; ++i) {
        resolved.poi_depth_map[i] = geometry_msgs.msg.Point.Resolve(msg.poi_depth_map[i]);
      }
    }
    else {
      resolved.poi_depth_map = []
    }

    return resolved;
    }
};

module.exports = {
  Request: PointsDepthMapRequest,
  Response: PointsDepthMapResponse,
  md5sum() { return '053187b90257a916cb98c3355d430439'; },
  datatype() { return 'tuni_whitegoods_msgs/PointsDepthMap'; }
};
