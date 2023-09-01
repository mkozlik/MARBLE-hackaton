# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: remote_control.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import std_pb2 as std__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='remote_control.proto',
  package='remotecontrol',
  syntax='proto3',
  serialized_options=_b('\n\036io.grpc.examples.remotecontrolB\rRemoteControlP\001\242\002\003HLW'),
  serialized_pb=_b('\n\x14remote_control.proto\x12\rremotecontrol\x1a\tstd.proto\"\x1f\n\x0c\x46orceRequest\x12\x0f\n\x07\x61\x64\x64ress\x18\x01 \x01(\t\"@\n\rForceResponse\x12\x0f\n\x07success\x18\x01 \x01(\x08\x12\x1e\n\x03pwm\x18\x02 \x01(\x0b\x32\x11.std.Float32Array2\\\n\rRemoteControl\x12K\n\nApplyForce\x12\x1b.remotecontrol.ForceRequest\x1a\x1c.remotecontrol.ForceResponse\"\x00\x30\x01\x42\x37\n\x1eio.grpc.examples.remotecontrolB\rRemoteControlP\x01\xa2\x02\x03HLWb\x06proto3')
  ,
  dependencies=[std__pb2.DESCRIPTOR,])




_FORCEREQUEST = _descriptor.Descriptor(
  name='ForceRequest',
  full_name='remotecontrol.ForceRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='address', full_name='remotecontrol.ForceRequest.address', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
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
  serialized_start=50,
  serialized_end=81,
)


_FORCERESPONSE = _descriptor.Descriptor(
  name='ForceResponse',
  full_name='remotecontrol.ForceResponse',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='success', full_name='remotecontrol.ForceResponse.success', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pwm', full_name='remotecontrol.ForceResponse.pwm', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
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
  serialized_start=83,
  serialized_end=147,
)

_FORCERESPONSE.fields_by_name['pwm'].message_type = std__pb2._FLOAT32ARRAY
DESCRIPTOR.message_types_by_name['ForceRequest'] = _FORCEREQUEST
DESCRIPTOR.message_types_by_name['ForceResponse'] = _FORCERESPONSE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ForceRequest = _reflection.GeneratedProtocolMessageType('ForceRequest', (_message.Message,), dict(
  DESCRIPTOR = _FORCEREQUEST,
  __module__ = 'remote_control_pb2'
  # @@protoc_insertion_point(class_scope:remotecontrol.ForceRequest)
  ))
_sym_db.RegisterMessage(ForceRequest)

ForceResponse = _reflection.GeneratedProtocolMessageType('ForceResponse', (_message.Message,), dict(
  DESCRIPTOR = _FORCERESPONSE,
  __module__ = 'remote_control_pb2'
  # @@protoc_insertion_point(class_scope:remotecontrol.ForceResponse)
  ))
_sym_db.RegisterMessage(ForceResponse)


DESCRIPTOR._options = None

_REMOTECONTROL = _descriptor.ServiceDescriptor(
  name='RemoteControl',
  full_name='remotecontrol.RemoteControl',
  file=DESCRIPTOR,
  index=0,
  serialized_options=None,
  serialized_start=149,
  serialized_end=241,
  methods=[
  _descriptor.MethodDescriptor(
    name='ApplyForce',
    full_name='remotecontrol.RemoteControl.ApplyForce',
    index=0,
    containing_service=None,
    input_type=_FORCEREQUEST,
    output_type=_FORCERESPONSE,
    serialized_options=None,
  ),
])
_sym_db.RegisterServiceDescriptor(_REMOTECONTROL)

DESCRIPTOR.services_by_name['RemoteControl'] = _REMOTECONTROL

# @@protoc_insertion_point(module_scope)
