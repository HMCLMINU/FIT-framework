# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from autoware_map_msgs/SignalLightArray.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import autoware_map_msgs.msg
import std_msgs.msg

class SignalLightArray(genpy.Message):
  _md5sum = "a70e2f53065c985af5dfda4d27a6eac1"
  _type = "autoware_map_msgs/SignalLightArray"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """# This consists of multiple SignalLight objects in a map. 

Header header
SignalLight[] data

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
MSG: autoware_map_msgs/SignalLight
# This represents each lamps in signal light in a map.

# COLOR_TYPES
uint8 RED = 1
uint8 GREEN = 2
uint8 YELLOW = 3
uint8 RED_FLASHING = 4
uint8 GREEN_FLASHING = 5
uint8 YELLOW_FLASHING = 6
uint8 UNKOWN = 7

# ARROW_TYPES
uint8 NONE = 0
uint8 UP = 1
uint8 UP_RIGHT = 2
uint8 RIGHT = 3
uint8 DOWN_RIGHT = 4
uint8 DOWN = 5
uint8 DOWN_LEFT = 6
uint8 LEFT = 7
uint8 UP_LEFT = 8
uint8 UNKNOWN = 9

# Id of this SignalLight object. Must be uniqe among all SignalLight objects. 
int32 signal_light_id

# Id of Signal object which this SignalLight object belongs to
int32 signal_id

# Id of point that represents the position of this lamp. 
int32 point_id

# Horizontal direction which this SignalLight is facing in [deg].
# Angle is calculated from North in clockwise direction. 
float64 horizontal_angle

# Vertical direction which this SignalLight is facing in [deg].
# Angle is calculated from vertical rising direction. 
float64 vertical_angle

# Color of the lamp
# Must be one of COLOR_TYPES
int32 color_type

# Direction of the arrow (if the lamp has direction)
# Must be one of ARROW_TYPES
int32 arrow_type
"""
  __slots__ = ['header','data']
  _slot_types = ['std_msgs/Header','autoware_map_msgs/SignalLight[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,data

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SignalLightArray, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.data is None:
        self.data = []
    else:
      self.header = std_msgs.msg.Header()
      self.data = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.data)
      buff.write(_struct_I.pack(length))
      for val1 in self.data:
        _x = val1
        buff.write(_get_struct_3i2d2i().pack(_x.signal_light_id, _x.signal_id, _x.point_id, _x.horizontal_angle, _x.vertical_angle, _x.color_type, _x.arrow_type))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.data is None:
        self.data = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.data = []
      for i in range(0, length):
        val1 = autoware_map_msgs.msg.SignalLight()
        _x = val1
        start = end
        end += 36
        (_x.signal_light_id, _x.signal_id, _x.point_id, _x.horizontal_angle, _x.vertical_angle, _x.color_type, _x.arrow_type,) = _get_struct_3i2d2i().unpack(str[start:end])
        self.data.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.data)
      buff.write(_struct_I.pack(length))
      for val1 in self.data:
        _x = val1
        buff.write(_get_struct_3i2d2i().pack(_x.signal_light_id, _x.signal_id, _x.point_id, _x.horizontal_angle, _x.vertical_angle, _x.color_type, _x.arrow_type))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.data is None:
        self.data = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.data = []
      for i in range(0, length):
        val1 = autoware_map_msgs.msg.SignalLight()
        _x = val1
        start = end
        end += 36
        (_x.signal_light_id, _x.signal_id, _x.point_id, _x.horizontal_angle, _x.vertical_angle, _x.color_type, _x.arrow_type,) = _get_struct_3i2d2i().unpack(str[start:end])
        self.data.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_3i2d2i = None
def _get_struct_3i2d2i():
    global _struct_3i2d2i
    if _struct_3i2d2i is None:
        _struct_3i2d2i = struct.Struct("<3i2d2i")
    return _struct_3i2d2i
