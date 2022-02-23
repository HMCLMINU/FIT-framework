# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from vector_map_msgs/Signal.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class Signal(genpy.Message):
  _md5sum = "a72afe3a758f79b5870ccafe3272b39c"
  _type = "vector_map_msgs/Signal"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# type
uint8 RED=1
uint8 BLUE=2 #Green (outside Japan)
uint8 YELLOW=3
uint8 PEDESTRIAN_RED=4
uint8 PEDESTRIAN_BLUE=5
uint8 OTHER=9

# Ver 1.00
int32 id
int32 vid
int32 plid
int32 type
int32 linkid

# left turn
uint8 RED_LEFT=21
uint8 BLUE_LEFT=22 #Green (outside Japan)
uint8 YELLOW_LEFT=23
"""
  # Pseudo-constants
  RED = 1
  BLUE = 2
  YELLOW = 3
  PEDESTRIAN_RED = 4
  PEDESTRIAN_BLUE = 5
  OTHER = 9
  RED_LEFT = 21
  BLUE_LEFT = 22
  YELLOW_LEFT = 23

  __slots__ = ['id','vid','plid','type','linkid']
  _slot_types = ['int32','int32','int32','int32','int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       id,vid,plid,type,linkid

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Signal, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.id is None:
        self.id = 0
      if self.vid is None:
        self.vid = 0
      if self.plid is None:
        self.plid = 0
      if self.type is None:
        self.type = 0
      if self.linkid is None:
        self.linkid = 0
    else:
      self.id = 0
      self.vid = 0
      self.plid = 0
      self.type = 0
      self.linkid = 0

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
      buff.write(_get_struct_5i().pack(_x.id, _x.vid, _x.plid, _x.type, _x.linkid))
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
      end += 20
      (_x.id, _x.vid, _x.plid, _x.type, _x.linkid,) = _get_struct_5i().unpack(str[start:end])
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
      buff.write(_get_struct_5i().pack(_x.id, _x.vid, _x.plid, _x.type, _x.linkid))
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
      end += 20
      (_x.id, _x.vid, _x.plid, _x.type, _x.linkid,) = _get_struct_5i().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_5i = None
def _get_struct_5i():
    global _struct_5i
    if _struct_5i is None:
        _struct_5i = struct.Struct("<5i")
    return _struct_5i
