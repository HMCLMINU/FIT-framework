# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from autoware_map_msgs/WaypointLaneRelation.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class WaypointLaneRelation(genpy.Message):
  _md5sum = "b8f61b7c5089da0aa24b1fd1cd929398"
  _type = "autoware_map_msgs/WaypointLaneRelation"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# This represents the connection between waypoints and lanes

# Id of refering Waypoint object
int32 waypoint_id

# Id of Lane object which refering waypoint belongs to 
int32 lane_id

# order of the waypoint in the lane. 
# e.g. starting waypoint of the lane will have order of 0. 
int32 order
"""
  __slots__ = ['waypoint_id','lane_id','order']
  _slot_types = ['int32','int32','int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       waypoint_id,lane_id,order

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(WaypointLaneRelation, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.waypoint_id is None:
        self.waypoint_id = 0
      if self.lane_id is None:
        self.lane_id = 0
      if self.order is None:
        self.order = 0
    else:
      self.waypoint_id = 0
      self.lane_id = 0
      self.order = 0

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
      buff.write(_get_struct_3i().pack(_x.waypoint_id, _x.lane_id, _x.order))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 12
      (_x.waypoint_id, _x.lane_id, _x.order,) = _get_struct_3i().unpack(str[start:end])
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
      buff.write(_get_struct_3i().pack(_x.waypoint_id, _x.lane_id, _x.order))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 12
      (_x.waypoint_id, _x.lane_id, _x.order,) = _get_struct_3i().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3i = None
def _get_struct_3i():
    global _struct_3i
    if _struct_3i is None:
        _struct_3i = struct.Struct("<3i")
    return _struct_3i
