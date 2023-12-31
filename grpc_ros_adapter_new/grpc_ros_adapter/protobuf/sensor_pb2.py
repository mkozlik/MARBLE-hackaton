# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: sensor.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import std_pb2 as std__pb2
import geometry_pb2 as geometry__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='sensor.proto',
  package='sensor',
  syntax='proto3',
  serialized_options=None,
  serialized_pb=_b('\n\x0csensor.proto\x12\x06sensor\x1a\tstd.proto\x1a\x0egeometry.proto\"\x90\x02\n\x03Imu\x12\x1b\n\x06header\x18\x01 \x01(\x0b\x32\x0b.std.Header\x12)\n\x0borientation\x18\x02 \x01(\x0b\x32\x14.geometry.Quaternion\x12\x1d\n\x15orientationCovariance\x18\x03 \x03(\x01\x12*\n\x0f\x61ngularVelocity\x18\x04 \x01(\x0b\x32\x11.geometry.Vector3\x12!\n\x19\x61ngularVelocityCovariance\x18\x05 \x03(\x01\x12-\n\x12linearAcceleration\x18\x06 \x01(\x0b\x32\x11.geometry.Vector3\x12$\n\x1clinearAccelerationCovariance\x18\x07 \x03(\x01\"\xf3\x01\n\x0cNavSatStatus\x12+\n\x06status\x18\x01 \x01(\x0e\x32\x1b.sensor.NavSatStatus.Status\x12-\n\x07service\x18\x02 \x01(\x0e\x32\x1c.sensor.NavSatStatus.Service\"B\n\x06Status\x12\x07\n\x03\x46IX\x10\x00\x12\x0c\n\x08SBAS_FIX\x10\x01\x12\x0c\n\x08GBAS_FIX\x10\x02\x12\x13\n\x06NO_FIX\x10\xff\xff\xff\xff\xff\xff\xff\xff\xff\x01\"C\n\x07Service\x12\x08\n\x04NONE\x10\x00\x12\x07\n\x03GPS\x10\x01\x12\x0b\n\x07GLONASS\x10\x02\x12\x0b\n\x07\x43OMPASS\x10\x04\x12\x0b\n\x07GALILEO\x10\x08\"\xa1\x01\n\tNavSatFix\x12\x1b\n\x06header\x18\x01 \x01(\x0b\x32\x0b.std.Header\x12$\n\x06status\x18\x02 \x01(\x0b\x32\x14.sensor.NavSatStatus\x12\x10\n\x08latitude\x18\x03 \x01(\x01\x12\x11\n\tlongitude\x18\x04 \x01(\x01\x12\x10\n\x08\x61ltitude\x18\x05 \x01(\x01\x12\x1a\n\x12positionCovariance\x18\x06 \x03(\x01\"\'\n\x07\x43hannel\x12\x0c\n\x04name\x18\x01 \x01(\t\x12\x0e\n\x06values\x18\x02 \x03(\x01\"\xd1\x01\n\nPointField\x12\x0c\n\x04name\x18\x01 \x01(\t\x12\x0e\n\x06offset\x18\x02 \x01(\r\x12-\n\x08\x64\x61tatype\x18\x03 \x01(\x0e\x32\x1b.sensor.PointField.DataType\x12\r\n\x05\x63ount\x18\x04 \x01(\r\"g\n\x08\x44\x61taType\x12\x08\n\x04INT8\x10\x00\x12\t\n\x05UINT8\x10\x01\x12\t\n\x05INT16\x10\x02\x12\n\n\x06UINT16\x10\x03\x12\t\n\x05INT32\x10\x04\x12\n\n\x06UINT32\x10\x05\x12\x0b\n\x07\x46LOAT32\x10\x06\x12\x0b\n\x07\x46LOAT64\x10\x07\"m\n\nPointCloud\x12\x1b\n\x06header\x18\x01 \x01(\x0b\x32\x0b.std.Header\x12\x1f\n\x06points\x18\x02 \x03(\x0b\x32\x0f.geometry.Point\x12!\n\x08\x63hannels\x18\x03 \x03(\x0b\x32\x0f.sensor.Channel\"\xdc\x01\n\x0bPointCloud2\x12\x1b\n\x06header\x18\x01 \x01(\x0b\x32\x0b.std.Header\x12\x15\n\rtimeInSeconds\x18\x02 \x01(\x01\x12\x0e\n\x06height\x18\x03 \x01(\r\x12\r\n\x05width\x18\x04 \x01(\r\x12\"\n\x06\x66ields\x18\x05 \x03(\x0b\x32\x12.sensor.PointField\x12\x13\n\x0bisBigEndian\x18\x06 \x01(\x08\x12\x11\n\tpointStep\x18\x07 \x01(\r\x12\x0f\n\x07rowStep\x18\x08 \x01(\r\x12\x0c\n\x04\x64\x61ta\x18\t \x01(\x0c\x12\x0f\n\x07isDense\x18\n \x01(\x08\"\x86\x01\n\x05Image\x12\x1b\n\x06header\x18\x01 \x01(\x0b\x32\x0b.std.Header\x12\x0e\n\x06height\x18\x02 \x01(\r\x12\r\n\x05width\x18\x03 \x01(\r\x12\x10\n\x08\x65ncoding\x18\x04 \x01(\t\x12\x13\n\x0bisBigEndian\x18\x05 \x01(\x08\x12\x0c\n\x04step\x18\x06 \x01(\r\x12\x0c\n\x04\x64\x61ta\x18\x07 \x01(\x0c\"L\n\x0f\x43ompressedImage\x12\x1b\n\x06header\x18\x01 \x01(\x0b\x32\x0b.std.Header\x12\x0e\n\x06\x66ormat\x18\x02 \x01(\t\x12\x0c\n\x04\x64\x61ta\x18\x03 \x01(\x0c\x62\x06proto3')
  ,
  dependencies=[std__pb2.DESCRIPTOR,geometry__pb2.DESCRIPTOR,])



_NAVSATSTATUS_STATUS = _descriptor.EnumDescriptor(
  name='Status',
  full_name='sensor.NavSatStatus.Status',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='FIX', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SBAS_FIX', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GBAS_FIX', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='NO_FIX', index=3, number=-1,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=435,
  serialized_end=501,
)
_sym_db.RegisterEnumDescriptor(_NAVSATSTATUS_STATUS)

_NAVSATSTATUS_SERVICE = _descriptor.EnumDescriptor(
  name='Service',
  full_name='sensor.NavSatStatus.Service',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='NONE', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GPS', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GLONASS', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='COMPASS', index=3, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GALILEO', index=4, number=8,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=503,
  serialized_end=570,
)
_sym_db.RegisterEnumDescriptor(_NAVSATSTATUS_SERVICE)

_POINTFIELD_DATATYPE = _descriptor.EnumDescriptor(
  name='DataType',
  full_name='sensor.PointField.DataType',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='INT8', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='UINT8', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INT16', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='UINT16', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INT32', index=4, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='UINT32', index=5, number=5,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FLOAT32', index=6, number=6,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FLOAT64', index=7, number=7,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=884,
  serialized_end=987,
)
_sym_db.RegisterEnumDescriptor(_POINTFIELD_DATATYPE)


_IMU = _descriptor.Descriptor(
  name='Imu',
  full_name='sensor.Imu',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='sensor.Imu.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='orientation', full_name='sensor.Imu.orientation', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='orientationCovariance', full_name='sensor.Imu.orientationCovariance', index=2,
      number=3, type=1, cpp_type=5, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='angularVelocity', full_name='sensor.Imu.angularVelocity', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='angularVelocityCovariance', full_name='sensor.Imu.angularVelocityCovariance', index=4,
      number=5, type=1, cpp_type=5, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='linearAcceleration', full_name='sensor.Imu.linearAcceleration', index=5,
      number=6, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='linearAccelerationCovariance', full_name='sensor.Imu.linearAccelerationCovariance', index=6,
      number=7, type=1, cpp_type=5, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=52,
  serialized_end=324,
)


_NAVSATSTATUS = _descriptor.Descriptor(
  name='NavSatStatus',
  full_name='sensor.NavSatStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='status', full_name='sensor.NavSatStatus.status', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='service', full_name='sensor.NavSatStatus.service', index=1,
      number=2, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _NAVSATSTATUS_STATUS,
    _NAVSATSTATUS_SERVICE,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=327,
  serialized_end=570,
)


_NAVSATFIX = _descriptor.Descriptor(
  name='NavSatFix',
  full_name='sensor.NavSatFix',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='sensor.NavSatFix.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='status', full_name='sensor.NavSatFix.status', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='latitude', full_name='sensor.NavSatFix.latitude', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='longitude', full_name='sensor.NavSatFix.longitude', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='altitude', full_name='sensor.NavSatFix.altitude', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='positionCovariance', full_name='sensor.NavSatFix.positionCovariance', index=5,
      number=6, type=1, cpp_type=5, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=573,
  serialized_end=734,
)


_CHANNEL = _descriptor.Descriptor(
  name='Channel',
  full_name='sensor.Channel',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='sensor.Channel.name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='values', full_name='sensor.Channel.values', index=1,
      number=2, type=1, cpp_type=5, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=736,
  serialized_end=775,
)


_POINTFIELD = _descriptor.Descriptor(
  name='PointField',
  full_name='sensor.PointField',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='sensor.PointField.name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='offset', full_name='sensor.PointField.offset', index=1,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='datatype', full_name='sensor.PointField.datatype', index=2,
      number=3, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='count', full_name='sensor.PointField.count', index=3,
      number=4, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _POINTFIELD_DATATYPE,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=778,
  serialized_end=987,
)


_POINTCLOUD = _descriptor.Descriptor(
  name='PointCloud',
  full_name='sensor.PointCloud',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='sensor.PointCloud.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='points', full_name='sensor.PointCloud.points', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='channels', full_name='sensor.PointCloud.channels', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=989,
  serialized_end=1098,
)


_POINTCLOUD2 = _descriptor.Descriptor(
  name='PointCloud2',
  full_name='sensor.PointCloud2',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='sensor.PointCloud2.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='timeInSeconds', full_name='sensor.PointCloud2.timeInSeconds', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='height', full_name='sensor.PointCloud2.height', index=2,
      number=3, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='width', full_name='sensor.PointCloud2.width', index=3,
      number=4, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='fields', full_name='sensor.PointCloud2.fields', index=4,
      number=5, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isBigEndian', full_name='sensor.PointCloud2.isBigEndian', index=5,
      number=6, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pointStep', full_name='sensor.PointCloud2.pointStep', index=6,
      number=7, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rowStep', full_name='sensor.PointCloud2.rowStep', index=7,
      number=8, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='data', full_name='sensor.PointCloud2.data', index=8,
      number=9, type=12, cpp_type=9, label=1,
      has_default_value=False, default_value=_b(""),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isDense', full_name='sensor.PointCloud2.isDense', index=9,
      number=10, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1101,
  serialized_end=1321,
)


_IMAGE = _descriptor.Descriptor(
  name='Image',
  full_name='sensor.Image',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='sensor.Image.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='height', full_name='sensor.Image.height', index=1,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='width', full_name='sensor.Image.width', index=2,
      number=3, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='encoding', full_name='sensor.Image.encoding', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='isBigEndian', full_name='sensor.Image.isBigEndian', index=4,
      number=5, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='step', full_name='sensor.Image.step', index=5,
      number=6, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='data', full_name='sensor.Image.data', index=6,
      number=7, type=12, cpp_type=9, label=1,
      has_default_value=False, default_value=_b(""),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1324,
  serialized_end=1458,
)


_COMPRESSEDIMAGE = _descriptor.Descriptor(
  name='CompressedImage',
  full_name='sensor.CompressedImage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='sensor.CompressedImage.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='format', full_name='sensor.CompressedImage.format', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='data', full_name='sensor.CompressedImage.data', index=2,
      number=3, type=12, cpp_type=9, label=1,
      has_default_value=False, default_value=_b(""),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1460,
  serialized_end=1536,
)

_IMU.fields_by_name['header'].message_type = std__pb2._HEADER
_IMU.fields_by_name['orientation'].message_type = geometry__pb2._QUATERNION
_IMU.fields_by_name['angularVelocity'].message_type = geometry__pb2._VECTOR3
_IMU.fields_by_name['linearAcceleration'].message_type = geometry__pb2._VECTOR3
_NAVSATSTATUS.fields_by_name['status'].enum_type = _NAVSATSTATUS_STATUS
_NAVSATSTATUS.fields_by_name['service'].enum_type = _NAVSATSTATUS_SERVICE
_NAVSATSTATUS_STATUS.containing_type = _NAVSATSTATUS
_NAVSATSTATUS_SERVICE.containing_type = _NAVSATSTATUS
_NAVSATFIX.fields_by_name['header'].message_type = std__pb2._HEADER
_NAVSATFIX.fields_by_name['status'].message_type = _NAVSATSTATUS
_POINTFIELD.fields_by_name['datatype'].enum_type = _POINTFIELD_DATATYPE
_POINTFIELD_DATATYPE.containing_type = _POINTFIELD
_POINTCLOUD.fields_by_name['header'].message_type = std__pb2._HEADER
_POINTCLOUD.fields_by_name['points'].message_type = geometry__pb2._POINT
_POINTCLOUD.fields_by_name['channels'].message_type = _CHANNEL
_POINTCLOUD2.fields_by_name['header'].message_type = std__pb2._HEADER
_POINTCLOUD2.fields_by_name['fields'].message_type = _POINTFIELD
_IMAGE.fields_by_name['header'].message_type = std__pb2._HEADER
_COMPRESSEDIMAGE.fields_by_name['header'].message_type = std__pb2._HEADER
DESCRIPTOR.message_types_by_name['Imu'] = _IMU
DESCRIPTOR.message_types_by_name['NavSatStatus'] = _NAVSATSTATUS
DESCRIPTOR.message_types_by_name['NavSatFix'] = _NAVSATFIX
DESCRIPTOR.message_types_by_name['Channel'] = _CHANNEL
DESCRIPTOR.message_types_by_name['PointField'] = _POINTFIELD
DESCRIPTOR.message_types_by_name['PointCloud'] = _POINTCLOUD
DESCRIPTOR.message_types_by_name['PointCloud2'] = _POINTCLOUD2
DESCRIPTOR.message_types_by_name['Image'] = _IMAGE
DESCRIPTOR.message_types_by_name['CompressedImage'] = _COMPRESSEDIMAGE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Imu = _reflection.GeneratedProtocolMessageType('Imu', (_message.Message,), dict(
  DESCRIPTOR = _IMU,
  __module__ = 'sensor_pb2'
  # @@protoc_insertion_point(class_scope:sensor.Imu)
  ))
_sym_db.RegisterMessage(Imu)

NavSatStatus = _reflection.GeneratedProtocolMessageType('NavSatStatus', (_message.Message,), dict(
  DESCRIPTOR = _NAVSATSTATUS,
  __module__ = 'sensor_pb2'
  # @@protoc_insertion_point(class_scope:sensor.NavSatStatus)
  ))
_sym_db.RegisterMessage(NavSatStatus)

NavSatFix = _reflection.GeneratedProtocolMessageType('NavSatFix', (_message.Message,), dict(
  DESCRIPTOR = _NAVSATFIX,
  __module__ = 'sensor_pb2'
  # @@protoc_insertion_point(class_scope:sensor.NavSatFix)
  ))
_sym_db.RegisterMessage(NavSatFix)

Channel = _reflection.GeneratedProtocolMessageType('Channel', (_message.Message,), dict(
  DESCRIPTOR = _CHANNEL,
  __module__ = 'sensor_pb2'
  # @@protoc_insertion_point(class_scope:sensor.Channel)
  ))
_sym_db.RegisterMessage(Channel)

PointField = _reflection.GeneratedProtocolMessageType('PointField', (_message.Message,), dict(
  DESCRIPTOR = _POINTFIELD,
  __module__ = 'sensor_pb2'
  # @@protoc_insertion_point(class_scope:sensor.PointField)
  ))
_sym_db.RegisterMessage(PointField)

PointCloud = _reflection.GeneratedProtocolMessageType('PointCloud', (_message.Message,), dict(
  DESCRIPTOR = _POINTCLOUD,
  __module__ = 'sensor_pb2'
  # @@protoc_insertion_point(class_scope:sensor.PointCloud)
  ))
_sym_db.RegisterMessage(PointCloud)

PointCloud2 = _reflection.GeneratedProtocolMessageType('PointCloud2', (_message.Message,), dict(
  DESCRIPTOR = _POINTCLOUD2,
  __module__ = 'sensor_pb2'
  # @@protoc_insertion_point(class_scope:sensor.PointCloud2)
  ))
_sym_db.RegisterMessage(PointCloud2)

Image = _reflection.GeneratedProtocolMessageType('Image', (_message.Message,), dict(
  DESCRIPTOR = _IMAGE,
  __module__ = 'sensor_pb2'
  # @@protoc_insertion_point(class_scope:sensor.Image)
  ))
_sym_db.RegisterMessage(Image)

CompressedImage = _reflection.GeneratedProtocolMessageType('CompressedImage', (_message.Message,), dict(
  DESCRIPTOR = _COMPRESSEDIMAGE,
  __module__ = 'sensor_pb2'
  # @@protoc_insertion_point(class_scope:sensor.CompressedImage)
  ))
_sym_db.RegisterMessage(CompressedImage)


# @@protoc_insertion_point(module_scope)
