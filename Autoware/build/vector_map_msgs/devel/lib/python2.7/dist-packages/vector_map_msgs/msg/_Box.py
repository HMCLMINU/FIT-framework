# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from vector_map_msgs/Box.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class Box(genpy.Message):
  _md5sum = "279dc10360643592a62c756918e5d27e"
  _type = "vector_map_msgs/Box"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# Ver 1.00
int32 bid
int32 pid1
int32 pid2
int32 pid3
int32 pid4
float64 height
"""
  __slots__ = ['bid','pid1','pid2','pid3','pid4','height']
  _slot_types = ['int32','int32','int32','int32','int32','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       bid,pid1,pid2,pid3,pid4,height

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Box, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.bid is None:
        self.bid = 0
      if self.pid1 is None:
        self.pid1 = 0
      if self.pid2 is None:
        self.pid2 = 0
      if self.pid3 is None:
        self.pid3 = 0
      if self.pid4 is None:
        self.pid4 = 0
      if self.height is None:
        self.height = 0.
    else:
      self.bid = 0
      self.pid1 = 0
      self.pid2 = 0
      self.pid3 = 0
      self.pid4 = 0
      self.height = 0.

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
      buff.write(_get_struct_5id().pack(_x.bid, _x.pid1, _x.pid2, _x.pid3, _x.pid4, _x.height))
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
      end += 28
      (_x.bid, _x.pid1, _x.pid2, _x.pid3, _x.pid4, _x.height,) = _get_struct_5id().unpack(str[start:end])
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
      buff.write(_get_struct_5id().pack(_x.bid, _x.pid1, _x.pid2, _x.pid3, _x.pid4, _x.height))
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
      end += 28
      (_x.bid, _x.pid1, _x.pid2, _x.pid3, _x.pid4, _x.height,) = _get_struct_5id().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_5id = None
def _get_struct_5id():
    global _struct_5id
    if _struct_5id is None:
        _struct_5id = struct.Struct("<5id")
    return _struct_5id
